from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from aivn_interfaces.msg import AisShip

from .ais_nmea_parser import AisDecoded, AisNmeaParser


@dataclass
class _StaticInfo:
    ship_name: str = ""
    call_sign: str = ""
    ship_type: int = 0
    updated_monotonic: float = 0.0


class AisTopicBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("ais_topic_bridge_node")

        self.declare_parameter(
            "input_topic_name", "/sensor_pack/navigation/novatel_isro_p2/w2k_60003"
        )
        self.declare_parameter("ais_topic_name", "/sensor_pack/external/ais/ship")
        self.declare_parameter("ais_frame_id", "ais")
        self.declare_parameter("ais_source_port", "w2k_60003")
        self.declare_parameter("ais_verbose", False)
        self.declare_parameter("ais_checksum_required", True)
        self.declare_parameter("ais_stale_static_info_sec", 600.0)
        self.declare_parameter("ais_debug_publish_reason", False)
        self.declare_parameter("ais_debug_fragment", False)
        self.declare_parameter("input_queue_size", 100)
        self.declare_parameter("output_queue_size", 100)

        self.input_topic_name = str(self.get_parameter("input_topic_name").value)
        self.topic_name = str(self.get_parameter("ais_topic_name").value)
        self.frame_id = str(self.get_parameter("ais_frame_id").value)
        self.source_port = str(self.get_parameter("ais_source_port").value)
        self.verbose = bool(self.get_parameter("ais_verbose").value)
        checksum_required = bool(self.get_parameter("ais_checksum_required").value)
        self.stale_static_info_sec = float(
            self.get_parameter("ais_stale_static_info_sec").value
        )
        self.debug_publish_reason = bool(
            self.get_parameter("ais_debug_publish_reason").value
        )
        self.debug_fragment = bool(self.get_parameter("ais_debug_fragment").value)
        input_queue_size = int(self.get_parameter("input_queue_size").value)
        output_queue_size = int(self.get_parameter("output_queue_size").value)

        parser_debug_cb = self._debug_fragment if self.debug_fragment else None
        self.parser = AisNmeaParser(
            checksum_required=checksum_required,
            debug_cb=parser_debug_cb,
        )
        self.publisher_ = self.create_publisher(AisShip, self.topic_name, output_queue_size)
        self.subscription_ = self.create_subscription(
            String,
            self.input_topic_name,
            self._input_callback,
            input_queue_size,
        )
        self._static_by_mmsi: Dict[int, _StaticInfo] = {}
        self.stats = {
            "rx_messages": 0,
            "rx_lines": 0,
            "decoded_ok": 0,
            "decoded_err": 0,
            "published": 0,
        }
        self._last_summary_time = time.monotonic()

        self.get_logger().info(
            "AIS topic bridge started: "
            f"{self.input_topic_name} -> {self.topic_name} "
            f"(frame_id={self.frame_id}, source_port={self.source_port})"
        )

    def _input_callback(self, msg: String) -> None:
        self.stats["rx_messages"] += 1
        raw_text = msg.data or ""
        for line in self._extract_lines(raw_text):
            self.stats["rx_lines"] += 1
            if self.verbose:
                print(f"ais_bridge_original_sentence: '{line}'")
            try:
                decoded = self.parser.parse_sentence(line)
                if decoded is None:
                    if self.debug_publish_reason:
                        self.get_logger().info(
                            "AIS bridge ignored input before publish: "
                            "unsupported message type or fragment assembly still in progress"
                        )
                    continue
                self._handle_decoded(decoded)
                self.stats["decoded_ok"] += 1
            except Exception as exc:
                self.stats["decoded_err"] += 1
                self.get_logger().warn(f"AIS bridge parse error: {exc} / raw={line}")

        self._log_summary()

    @staticmethod
    def _extract_lines(raw_text: str) -> list[str]:
        return [line.strip() for line in raw_text.splitlines() if line.strip()]

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
                    "AIS bridge dropped message before publish: "
                    f"msg_id={decoded.ais_message_id} mmsi={decoded.mmsi} "
                    f"position_valid={decoded.position_valid} static_valid={decoded.static_valid}"
                )
            return

        out_msg = self._to_msg(decoded)
        if self.debug_publish_reason:
            self.get_logger().info(
                "AIS bridge publishing message: "
                f"msg_id={out_msg.ais_message_id} mmsi={out_msg.mmsi} "
                f"position_valid={out_msg.position_valid} static_valid={out_msg.static_valid} "
                f"lat={out_msg.lat:.6f} lon={out_msg.lon:.6f}"
            )
        self.publisher_.publish(out_msg)
        self.stats["published"] += 1

    def _to_msg(self, decoded: AisDecoded) -> AisShip:
        msg = AisShip()
        stamp = self.get_clock().now()
        msg.header.stamp = stamp.to_msg()
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
        msg.receiving_time_unix = int(stamp.nanoseconds // 1_000_000_000)
        msg.original_sentence = decoded.original_sentence
        msg.source_port = self.source_port
        return msg

    def _log_summary(self) -> None:
        now = time.monotonic()
        if not self.verbose or now - self._last_summary_time < 5.0:
            return
        self._last_summary_time = now
        self.get_logger().info(
            "AIS topic bridge summary: "
            f"messages={self.stats['rx_messages']} "
            f"lines={self.stats['rx_lines']} "
            f"decoded_ok={self.stats['decoded_ok']} "
            f"decoded_err={self.stats['decoded_err']} "
            f"published={self.stats['published']}"
        )

    def _debug_fragment(self, text: str) -> None:
        self.get_logger().info(f"AIS fragment: {text}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AisTopicBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
