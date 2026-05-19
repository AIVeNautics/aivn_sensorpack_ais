from __future__ import annotations

from dataclasses import dataclass
import errno
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
        self.declare_parameter("ais_data_bits", 8)
        self.declare_parameter("ais_parity", "N")
        self.declare_parameter("ais_stop_bits", 1)
        self.declare_parameter("ais_xonxoff", False)
        self.declare_parameter("ais_rtscts", False)
        self.declare_parameter("ais_dsrdtr", False)
        self.declare_parameter("ais_poll_period_sec", 0.005)
        self.declare_parameter("ais_read_size", 8192)
        self.declare_parameter("ais_reconnect_sec", 2.0)
        self.declare_parameter("ais_stale_static_info_sec", 600.0)
        self.declare_parameter("ais_no_data_warn_sec", 10.0)
        self.declare_parameter("ais_no_nmea_sync_warn_sec", 3.0)
        self.declare_parameter("ais_debug_hex_dump", False)
        self.declare_parameter("ais_debug_hex_limit", 64)
        self.declare_parameter("ais_debug_publish_reason", False)
        self.declare_parameter("ais_debug_fragment", False)

        self.port = self.get_parameter("ais_serial_port_name").value
        self.baud = int(self.get_parameter("ais_baud_rate").value)
        self.topic_name = self.get_parameter("ais_topic_name").value
        self.frame_id = self.get_parameter("ais_frame_id").value
        self.verbose = bool(self.get_parameter("ais_verbose").value)
        checksum_required = bool(self.get_parameter("ais_checksum_required").value)
        self.data_bits = self._parse_data_bits(int(self.get_parameter("ais_data_bits").value))
        self.parity = self._parse_parity(str(self.get_parameter("ais_parity").value))
        self.stop_bits = self._parse_stop_bits(float(self.get_parameter("ais_stop_bits").value))
        self.xonxoff = bool(self.get_parameter("ais_xonxoff").value)
        self.rtscts = bool(self.get_parameter("ais_rtscts").value)
        self.dsrdtr = bool(self.get_parameter("ais_dsrdtr").value)
        poll_period = float(self.get_parameter("ais_poll_period_sec").value)
        self.read_size = int(self.get_parameter("ais_read_size").value)
        self.reconnect_sec = float(self.get_parameter("ais_reconnect_sec").value)
        self.stale_static_info_sec = float(self.get_parameter("ais_stale_static_info_sec").value)
        self.no_data_warn_sec = float(self.get_parameter("ais_no_data_warn_sec").value)
        self.no_nmea_sync_warn_sec = float(self.get_parameter("ais_no_nmea_sync_warn_sec").value)
        self.debug_hex_dump = bool(self.get_parameter("ais_debug_hex_dump").value)
        self.debug_hex_limit = int(self.get_parameter("ais_debug_hex_limit").value)
        self.debug_publish_reason = bool(self.get_parameter("ais_debug_publish_reason").value)
        self.debug_fragment = bool(self.get_parameter("ais_debug_fragment").value)

        parser_debug_cb = self._debug_fragment if self.debug_fragment else None
        self.parser = AisNmeaParser(checksum_required=checksum_required, debug_cb=parser_debug_cb)
        self.publisher_ = self.create_publisher(AisShip, self.topic_name, 100)
        self.ser: Optional[serial.Serial] = None
        self._last_open_attempt = 0.0
        self._raw_buf = bytearray()
        self._static_by_mmsi: Dict[int, _StaticInfo] = {}
        self._last_summary_time = time.monotonic()
        self._opened_monotonic = 0.0
        self._last_rx_monotonic = 0.0
        self._last_no_data_warn_monotonic = 0.0
        self._non_nmea_since_monotonic = 0.0
        self._last_no_nmea_sync_warn_monotonic = 0.0
        self.stats = {
            "rx_bytes": 0,
            "rx_sentences": 0,
            "decoded_ok": 0,
            "decoded_err": 0,
            "published": 0,
        }

        self.timer = self.create_timer(poll_period, self._poll_serial)
        self.get_logger().info(
            "AIS-only serial node publishing "
            f"{self.topic_name}; port={self.port}, baud={self.baud}, "
            f"data_bits={self._data_bits_label()}, parity={self._parity_label()}, "
            f"stop_bits={self._stop_bits_label()}, xonxoff={self.xonxoff}, "
            f"rtscts={self.rtscts}, dsrdtr={self.dsrdtr}"
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
                bytesize=self.data_bits,
                parity=self.parity,
                stopbits=self.stop_bits,
                timeout=0,
                xonxoff=self.xonxoff,
                rtscts=self.rtscts,
                dsrdtr=self.dsrdtr,
            )
            now = time.monotonic()
            self._opened_monotonic = now
            self._last_rx_monotonic = now
            self._last_no_data_warn_monotonic = 0.0
            self._non_nmea_since_monotonic = 0.0
            self._last_no_nmea_sync_warn_monotonic = 0.0
            self.get_logger().info(
                "Opened AIS serial: "
                f"{self.port} ({self.baud} {self._data_bits_label()}"
                f"{self._parity_label()}{self._stop_bits_label()}, "
                f"xonxoff={self.xonxoff}, rtscts={self.rtscts}, dsrdtr={self.dsrdtr})"
            )
            return True
        except Exception as exc:
            self.ser = None
            if self._is_permission_error(exc):
                self.get_logger().warn(
                    "AIS serial open failed due to permissions: "
                    f"{exc}. Check that the current user can access {self.port} "
                    "(for example, by adding the user to 'dialout' or applying the correct udev rule)."
                )
            else:
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
        self._opened_monotonic = 0.0
        self._last_rx_monotonic = 0.0
        self._last_no_data_warn_monotonic = 0.0
        self._non_nmea_since_monotonic = 0.0
        self._last_no_nmea_sync_warn_monotonic = 0.0

    def _poll_serial(self) -> None:
        if not self._open_serial_if_needed():
            return

        try:
            data = self.ser.read(self.read_size)
            if data:
                self._raw_buf += data
                self.stats["rx_bytes"] += len(data)
                self._last_rx_monotonic = time.monotonic()
                self._log_hex_dump(data)
                self._track_nmea_sync_state()

            for sentence_bytes in self._extract_complete_sentences():
                self.stats["rx_sentences"] += 1
                sentence = sentence_bytes.decode("ascii", errors="ignore")
                if self.verbose:
                    print(f"ais_original_sentence: '{sentence}'")
                try:
                    decoded = self.parser.parse_sentence(sentence)
                    if decoded is None:
                        if self.debug_publish_reason:
                            self.get_logger().info(
                                "AIS sentence ignored before publish: "
                                "unsupported message type or fragment assembly still in progress"
                            )
                        continue
                    self._handle_decoded(decoded)
                    self.stats["decoded_ok"] += 1
                except Exception as exc:
                    self.stats["decoded_err"] += 1
                    self.get_logger().warn(f"AIS parse error: {exc} / raw={sentence}")

            self._log_summary()
            self._log_no_data_warning()
            self._log_no_nmea_sync_warning()

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

    @staticmethod
    def _is_permission_error(exc: Exception) -> bool:
        err_no = getattr(exc, "errno", None)
        if err_no == errno.EACCES:
            return True
        return "Permission denied" in str(exc)

    @staticmethod
    def _parse_data_bits(value: int) -> int:
        mapping = {
            5: serial.FIVEBITS,
            6: serial.SIXBITS,
            7: serial.SEVENBITS,
            8: serial.EIGHTBITS,
        }
        if value not in mapping:
            raise ValueError(f"unsupported ais_data_bits: {value}")
        return mapping[value]

    @staticmethod
    def _parse_parity(value: str) -> str:
        normalized = (value or "").strip().upper()
        mapping = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
            "M": serial.PARITY_MARK,
            "S": serial.PARITY_SPACE,
        }
        if normalized not in mapping:
            raise ValueError(f"unsupported ais_parity: {value!r}")
        return mapping[normalized]

    @staticmethod
    def _parse_stop_bits(value: float) -> float:
        mapping = {
            1.0: serial.STOPBITS_ONE,
            1.5: serial.STOPBITS_ONE_POINT_FIVE,
            2.0: serial.STOPBITS_TWO,
        }
        normalized = round(float(value), 1)
        if normalized not in mapping:
            raise ValueError(f"unsupported ais_stop_bits: {value}")
        return mapping[normalized]

    def _data_bits_label(self) -> int:
        reverse = {
            serial.FIVEBITS: 5,
            serial.SIXBITS: 6,
            serial.SEVENBITS: 7,
            serial.EIGHTBITS: 8,
        }
        return reverse[self.data_bits]

    def _parity_label(self) -> str:
        reverse = {
            serial.PARITY_NONE: "N",
            serial.PARITY_EVEN: "E",
            serial.PARITY_ODD: "O",
            serial.PARITY_MARK: "M",
            serial.PARITY_SPACE: "S",
        }
        return reverse[self.parity]

    def _stop_bits_label(self) -> str:
        reverse = {
            serial.STOPBITS_ONE: "1",
            serial.STOPBITS_ONE_POINT_FIVE: "1.5",
            serial.STOPBITS_TWO: "2",
        }
        return reverse[self.stop_bits]

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
            if self.debug_publish_reason:
                self.get_logger().info(
                    "AIS decoded message dropped before publish: "
                    f"msg_id={decoded.ais_message_id} mmsi={decoded.mmsi} "
                    f"position_valid={decoded.position_valid} static_valid={decoded.static_valid}"
                )
            return

        msg = self._to_msg(decoded)
        if self.debug_publish_reason:
            self.get_logger().info(
                "AIS publishing message: "
                f"msg_id={msg.ais_message_id} mmsi={msg.mmsi} "
                f"position_valid={msg.position_valid} static_valid={msg.static_valid} "
                f"lat={msg.lat:.6f} lon={msg.lon:.6f}"
            )
        self.publisher_.publish(msg)
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

    def _log_hex_dump(self, data: bytes) -> None:
        if not self.debug_hex_dump or not data:
            return

        limit = max(1, self.debug_hex_limit)
        preview = data[:limit]
        suffix = " ..." if len(data) > limit else ""
        ascii_preview = "".join(chr(ch) if 32 <= ch <= 126 else "." for ch in preview)
        self.get_logger().info(
            "AIS raw bytes: "
            f"count={len(data)} preview_hex={preview.hex(' ')} "
            f"preview_ascii={ascii_preview!r}{suffix}"
        )

    def _track_nmea_sync_state(self) -> None:
        if b"!" in self._raw_buf:
            self._non_nmea_since_monotonic = 0.0
            self._last_no_nmea_sync_warn_monotonic = 0.0
            return

        if self._raw_buf and self._non_nmea_since_monotonic <= 0.0:
            self._non_nmea_since_monotonic = time.monotonic()

    def _log_no_data_warning(self) -> None:
        if self.no_data_warn_sec <= 0.0 or self.ser is None or not self.ser.is_open:
            return

        now = time.monotonic()
        if self._opened_monotonic <= 0.0:
            return
        if now - self._opened_monotonic < self.no_data_warn_sec:
            return
        if now - self._last_rx_monotonic < self.no_data_warn_sec:
            return
        if (
            self._last_no_data_warn_monotonic > 0.0
            and now - self._last_no_data_warn_monotonic < self.no_data_warn_sec
        ):
            return

        self._last_no_data_warn_monotonic = now
        self.get_logger().warn(
            "AIS serial is open but no bytes have been received for "
            f"{now - self._last_rx_monotonic:.1f}s on {self.port}. "
            "Check the live serial stream, wiring, and port selection."
        )

    def _log_no_nmea_sync_warning(self) -> None:
        if self.no_nmea_sync_warn_sec <= 0.0 or self.ser is None or not self.ser.is_open:
            return
        if not self._raw_buf or self._non_nmea_since_monotonic <= 0.0:
            return

        now = time.monotonic()
        if now - self._non_nmea_since_monotonic < self.no_nmea_sync_warn_sec:
            return
        if (
            self._last_no_nmea_sync_warn_monotonic > 0.0
            and now - self._last_no_nmea_sync_warn_monotonic < self.no_nmea_sync_warn_sec
        ):
            return

        self._last_no_nmea_sync_warn_monotonic = now
        preview = bytes(self._raw_buf[: max(1, self.debug_hex_limit)])
        ascii_preview = "".join(chr(ch) if 32 <= ch <= 126 else "." for ch in preview)
        self.get_logger().warn(
            "AIS serial is receiving bytes but no NMEA '!' sentence start has been observed for "
            f"{now - self._non_nmea_since_monotonic:.1f}s on {self.port}. "
            "This usually means the serial format is wrong or the device is not outputting AIS NMEA "
            f"(for example, a gateway transfer mode). preview_hex={preview.hex(' ')} "
            f"preview_ascii={ascii_preview!r}"
        )

    def _debug_fragment(self, message: str) -> None:
        self.get_logger().info(f"AIS fragment debug: {message}")


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
