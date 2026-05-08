from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
import serial

from aivn_interfaces.msg import AisShip

from .ais_nmea_parser import AisDecoded, AisNmeaParser


@dataclass
class _StaticInfo:
    ship_name: str = ""
    call_sign: str = ""
    ship_type: int = 0
    updated_monotonic: float = 0.0


class AisSerialNode(Node):
    def __init__(self) -> None:
        super().__init__("ais_serial_node")

        self.declare_parameter("ais_serial_port_name", "/dev/ttyUSB0")
        self.declare_parameter("ais_baud_rate", 38400)
        self.declare_parameter("ais_topic_name", "/edge_server/external/ais/ship")
        self.declare_parameter("ais_frame_id", "ais")
        self.declare_parameter("ais_verbose", False)
        self.declare_parameter("ais_checksum_required", True)
        self.declare_parameter("ais_poll_period_sec", 0.005)
        self.declare_parameter("ais_read_size", 8192)
        self.declare_parameter("ais_reconnect_sec", 2.0)
        self.declare_parameter("ais_stale_static_info_sec", 600.0)

        self.port = self.get_parameter("ais_serial_port_name").value
        self.baud = int(self.get_parameter("ais_baud_rate").value)
        self.topic_name = self.get_parameter("ais_topic_name").value
        self.frame_id = self.get_parameter("ais_frame_id").value
        self.verbose = bool(self.get_parameter("ais_verbose").value)
        checksum_required = bool(self.get_parameter("ais_checksum_required").value)
        poll_period = float(self.get_parameter("ais_poll_period_sec").value)
        self.read_size = int(self.get_parameter("ais_read_size").value)
        self.reconnect_sec = float(self.get_parameter("ais_reconnect_sec").value)
        self.stale_static_info_sec = float(self.get_parameter("ais_stale_static_info_sec").value)

        self.parser = AisNmeaParser(checksum_required=checksum_required)
        self.publisher_ = self.create_publisher(AisShip, self.topic_name, 100)
        self.ser: Optional[serial.Serial] = None
        self._last_open_attempt = 0.0
        self._raw_buf = bytearray()
        self._static_by_mmsi: Dict[int, _StaticInfo] = {}
        self._last_summary_time = time.monotonic()
        self.stats = {
            "rx_bytes": 0,
            "rx_sentences": 0,
            "decoded_ok": 0,
            "decoded_err": 0,
            "published": 0,
        }

        self.timer = self.create_timer(poll_period, self._poll_serial)
        self.get_logger().info(
            f"AIS-only serial node publishing {self.topic_name}; port={self.port}, baud={self.baud}"
        )

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()

    def _open_serial_if_needed(self) -> bool:
        if self.ser is not None and self.ser.is_open:
            return True

        now = time.monotonic()
        if now - self._last_open_attempt < self.reconnect_sec:
            return False
        self._last_open_attempt = now

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self.get_logger().info(f"Opened AIS serial: {self.port} ({self.baud} 8N1)")
            return True
        except Exception as exc:
            self.ser = None
            self.get_logger().warn(f"AIS serial open failed: {exc}")
            return False

    def _close_serial(self) -> None:
        if self.ser is None:
            return
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _poll_serial(self) -> None:
        if not self._open_serial_if_needed():
            return

        try:
            data = self.ser.read(self.read_size)
            if data:
                self._raw_buf += data
                self.stats["rx_bytes"] += len(data)

            for sentence_bytes in self._extract_complete_sentences():
                self.stats["rx_sentences"] += 1
                sentence = sentence_bytes.decode("ascii", errors="ignore")
                if self.verbose:
                    print(f"ais_original_sentence: '{sentence}'")
                try:
                    decoded = self.parser.parse_sentence(sentence)
                    if decoded is None:
                        continue
                    self._handle_decoded(decoded)
                    self.stats["decoded_ok"] += 1
                except Exception as exc:
                    self.stats["decoded_err"] += 1
                    self.get_logger().warn(f"AIS parse error: {exc} / raw={sentence}")

            self._log_summary()

        except Exception as exc:
            self.get_logger().error(f"AIS serial read error: {exc}")
            self._close_serial()

    def _extract_complete_sentences(self):
        out = []
        buf = self._raw_buf
        while True:
            start = buf.find(b"!")
            if start == -1:
                buf.clear()
                break
            if start > 0:
                del buf[:start]

            star = buf.find(b"*", 1)
            if star == -1:
                break
            if star + 2 >= len(buf):
                break

            checksum = buf[star + 1:star + 3]
            if not self._is_hex_pair(checksum):
                del buf[:1]
                continue

            end = star + 3
            sentence = bytes(buf[:end])
            while end < len(buf) and buf[end] in (10, 13):
                end += 1
            del buf[:end]
            out.append(sentence)
        return out

    @staticmethod
    def _is_hex_pair(raw: bytes) -> bool:
        if len(raw) != 2:
            return False
        return all(ch in b"0123456789ABCDEFabcdef" for ch in raw)

    def _handle_decoded(self, decoded: AisDecoded) -> None:
        now_monotonic = time.monotonic()

        if decoded.static_valid:
            current = self._static_by_mmsi.get(decoded.mmsi, _StaticInfo())
            if decoded.ship_name:
                current.ship_name = decoded.ship_name
            if decoded.call_sign:
                current.call_sign = decoded.call_sign
            if decoded.ship_type:
                current.ship_type = decoded.ship_type
            current.updated_monotonic = now_monotonic
            self._static_by_mmsi[decoded.mmsi] = current

        static_info = self._static_by_mmsi.get(decoded.mmsi)
        if static_info and now_monotonic - static_info.updated_monotonic <= self.stale_static_info_sec:
            if not decoded.ship_name:
                decoded.ship_name = static_info.ship_name
            if not decoded.call_sign:
                decoded.call_sign = static_info.call_sign
            if not decoded.ship_type:
                decoded.ship_type = static_info.ship_type
            decoded.static_valid = decoded.static_valid or bool(
                decoded.ship_name or decoded.call_sign or decoded.ship_type
            )

        if not decoded.position_valid and not decoded.static_valid:
            return

        self.publisher_.publish(self._to_msg(decoded))
        self.stats["published"] += 1

    def _to_msg(self, decoded: AisDecoded) -> AisShip:
        msg = AisShip()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.ais_message_id = int(decoded.ais_message_id)
        msg.mmsi = int(decoded.mmsi)
        msg.ship_id = decoded.ship_id or str(decoded.mmsi)
        msg.ship_name = decoded.ship_name
        msg.call_sign = decoded.call_sign
        msg.ship_type = int(decoded.ship_type)
        msg.navigation_status = int(decoded.navigation_status)
        msg.lat = float(decoded.lat)
        msg.lon = float(decoded.lon)
        msg.sog = float(decoded.sog)
        msg.cog = float(decoded.cog)
        msg.heading = int(decoded.heading)
        msg.position_valid = bool(decoded.position_valid)
        msg.static_valid = bool(decoded.static_valid)
        msg.receiving_time_unix = int(now.nanoseconds // 1_000_000_000)
        msg.original_sentence = decoded.original_sentence
        msg.source_port = self.port
        return msg

    def _log_summary(self) -> None:
        now = time.monotonic()
        if not self.verbose or now - self._last_summary_time < 5.0:
            return
        self._last_summary_time = now
        self.get_logger().info(
            "AIS summary: "
            f"bytes={self.stats['rx_bytes']} "
            f"sentences={self.stats['rx_sentences']} "
            f"decoded_ok={self.stats['decoded_ok']} "
            f"decoded_err={self.stats['decoded_err']} "
            f"published={self.stats['published']}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AisSerialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
