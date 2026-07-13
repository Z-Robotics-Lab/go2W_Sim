#!/usr/bin/env python3
"""Turn manipulation status messages into standard RViz text markers.

This process is visualization-only: it consumes PiPER proprioceptive execution
state and the Z-Manip perception diagnostic contract.  It never subscribes to
task-object or robot ground-truth topics and never publishes a control command.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from diagnostic_level import diagnostic_level_to_int


DEFAULT_CONTRACT = "/ws/manipulation_rviz_topics.json"

DIAGNOSTIC_OK = diagnostic_level_to_int(DiagnosticStatus.OK)
DIAGNOSTIC_WARN = diagnostic_level_to_int(DiagnosticStatus.WARN)
DIAGNOSTIC_ERROR = diagnostic_level_to_int(DiagnosticStatus.ERROR)
DIAGNOSTIC_STALE = diagnostic_level_to_int(DiagnosticStatus.STALE)


class ManipRvizBridge(Node):
    """Publish latched, view-facing status text using standard RViz messages."""

    def __init__(self) -> None:
        super().__init__("z_manip_rviz_bridge")
        contract_path = Path(os.environ.get("GO2W_RVIZ_TOPICS_CONFIG", DEFAULT_CONTRACT))
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        topics = contract["topics"]

        defaults = {
            "status_frame": contract.get("status_frame", "base_link"),
            "piper_status_topic": topics["piper_execution_status"],
            "perception_status_topic": topics["perception_diagnostics"],
            "perception_valid_topic": topics["perception_valid"],
            "piper_marker_topic": topics["piper_execution_status_marker"],
            "perception_marker_topic": topics["perception_status_marker"],
            "status_stale_s": 2.0,
            "publish_period_s": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if float(self.get_parameter("status_stale_s").value) <= 0.0:
            raise ValueError("status_stale_s must be positive")
        if float(self.get_parameter("publish_period_s").value) <= 0.0:
            raise ValueError("publish_period_s must be positive")

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._piper_pub = self.create_publisher(
            Marker,
            self._param("piper_marker_topic"),
            latched,
        )
        self._perception_pub = self.create_publisher(
            Marker,
            self._param("perception_marker_topic"),
            latched,
        )
        self.create_subscription(
            String,
            self._param("piper_status_topic"),
            self._on_piper_status,
            reliable,
        )
        self.create_subscription(
            DiagnosticArray,
            self._param("perception_status_topic"),
            self._on_perception_status,
            reliable,
        )
        self.create_subscription(
            Bool,
            self._param("perception_valid_topic"),
            self._on_perception_valid,
            reliable,
        )
        self._piper_text = "waiting for /piper/execution_status"
        self._piper_at: float | None = None
        self._perception_text = "waiting for Z-Manip perception diagnostics"
        self._perception_level = DIAGNOSTIC_STALE
        self._perception_valid: bool | None = None
        self._perception_at: float | None = None
        self.create_timer(
            float(self.get_parameter("publish_period_s").value),
            self._publish,
        )
        self.get_logger().info("RViz status bridge ready (measured status only; no GT)")

    def _param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _on_piper_status(self, message: String) -> None:
        text = " ".join(message.data.strip().split())
        self._piper_text = text[:240] if text else "empty execution status"
        self._piper_at = time.monotonic()

    def _on_perception_status(self, message: DiagnosticArray) -> None:
        if not message.status:
            self._perception_text = "empty perception DiagnosticArray"
            self._perception_level = DIAGNOSTIC_ERROR
            self._perception_at = time.monotonic()
            return
        status = max(
            message.status,
            key=lambda item: diagnostic_level_to_int(item.level),
        )
        details = {
            item.key: item.value
            for item in status.values
            if item.key in ("phase", "target_label", "failure", "track_id") and item.value
        }
        suffix = " ".join(f"{key}={value}" for key, value in details.items())
        self._perception_text = f"{status.message} {suffix}".strip()[:240]
        self._perception_level = diagnostic_level_to_int(status.level)
        self._perception_at = time.monotonic()

    def _on_perception_valid(self, message: Bool) -> None:
        self._perception_valid = bool(message.data)

    def _publish(self) -> None:
        now = time.monotonic()
        stale_s = float(self.get_parameter("status_stale_s").value)

        piper_stale = self._piper_at is None or now - self._piper_at > stale_s
        piper_text = f"PiPER: {self._piper_text}"
        piper_color = self._piper_color(self._piper_text, stale=piper_stale)
        self._piper_pub.publish(
            self._text_marker(0, piper_text, 1.35, piper_color),
        )

        perception_stale = (
            self._perception_at is None or now - self._perception_at > stale_s
        )
        validity = (
            "unknown" if self._perception_valid is None
            else ("true" if self._perception_valid else "false")
        )
        perception_text = f"Perception valid={validity}: {self._perception_text}"
        level = DIAGNOSTIC_STALE if perception_stale else self._perception_level
        self._perception_pub.publish(
            self._text_marker(1, perception_text, 1.18, self._diagnostic_color(level)),
        )

    def _text_marker(
        self,
        marker_id: int,
        text: str,
        height: float,
        color: tuple[float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._param("status_frame")
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "z_manip_status"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.1
        marker.pose.position.y = 0.0
        marker.pose.position.z = height
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.095
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.text = text
        return marker

    @staticmethod
    def _piper_color(text: str, *, stale: bool) -> tuple[float, float, float]:
        if stale:
            return 0.65, 0.65, 0.65
        lowered = text.lower()
        if any(word in lowered for word in ("reject", "error", "failed", "cancel")):
            return 1.0, 0.18, 0.12
        if any(word in lowered for word in ("execut", "active", "running")):
            return 1.0, 0.72, 0.08
        return 0.18, 1.0, 0.35

    @staticmethod
    def _diagnostic_color(level: int) -> tuple[float, float, float]:
        level = diagnostic_level_to_int(level)
        if level == DIAGNOSTIC_OK:
            return 0.18, 1.0, 0.35
        if level == DIAGNOSTIC_WARN:
            return 1.0, 0.72, 0.08
        if level == DIAGNOSTIC_ERROR:
            return 1.0, 0.18, 0.12
        return 0.65, 0.65, 0.65


def main() -> None:
    rclpy.init()
    node = ManipRvizBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
