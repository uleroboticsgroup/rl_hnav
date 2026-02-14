#!/usr/bin/env python3
"""
scan_restamper.py

Purpose
-------
Some drivers / converters publish LaserScan timestamps that are "too old"
(PC time vs robot time drift, or sensor clock not synced). Nav2 and slam_toolbox
use TF message_filters and will drop LaserScan messages if their timestamps fall
outside the TF buffer window.

This node republishes LaserScan while optionally overriding header.stamp with
the local ROS clock time (Node clock). All frames and scan fields remain the same.

Recommended pipeline
--------------------
PointCloud2 -> pointcloud_to_laserscan -> /scan_raw   (stamp may be old)
 /scan_raw  -> scan_restamper        -> /scan         (stamp becomes now)

Parameters
----------
- input_scan_topic  (string): Input scan topic (default: /scan_raw)
- output_scan_topic (string): Output scan topic (default: /scan)
- override_stamp    (bool)  : If true, replace header.stamp with now() (default: true)

QoS strategy
------------
LaserScan is typically sensor-like traffic (often BEST_EFFORT).
We use a sensor-style QoS for both subscription and publisher to avoid blocking
and to keep latency low.

If you encounter "no messages received", check QoS mismatch with:
  ros2 topic info -v /scan_raw
and adjust reliability accordingly.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    """Republish LaserScan with a refreshed timestamp to avoid TF filter drops."""

    def __init__(self) -> None:
        super().__init__("scan_restamper")

        # -------------------------
        # Parameters
        # -------------------------
        self.input_scan_topic: str = (
            self.declare_parameter("input_scan_topic", "/scan_raw").value
        )
        self.output_scan_topic: str = (
            self.declare_parameter("output_scan_topic", "/scan").value
        )
        self.override_stamp: bool = (
            self.declare_parameter("override_stamp", True).value
        )

        # -------------------------
        # QoS (sensor-like, low latency)
        # -------------------------
        # Many LiDAR / scan topics are BEST_EFFORT with KEEP_LAST.
        # Using the same QoS for pub/sub keeps the behavior consistent.
        self.qos: QoSProfile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher: output /scan
        self.pub = self.create_publisher(LaserScan, self.output_scan_topic, self.qos)

        # Subscriber: input /scan_raw
        self.sub = self.create_subscription(
            LaserScan, self.input_scan_topic, self._cb, self.qos
        )

        # Log configuration once at startup
        self.get_logger().info(
            f"[scan_restamper] input='{self.input_scan_topic}' -> output='{self.output_scan_topic}', "
            f"override_stamp={self.override_stamp}, qos=BEST_EFFORT depth=10"
        )

    def _cb(self, msg: LaserScan) -> None:
        """
        Callback for incoming LaserScan.

        We create a new LaserScan message to avoid mutating the incoming 'msg'
        (which may be shared internally by rclpy).
        """
        out = LaserScan()

        # -------------------------
        # Header
        # -------------------------
        out.header = msg.header  # copies header struct (frame_id preserved)

        # Override timestamp if requested (main purpose of this node)
        if self.override_stamp:
            out.header.stamp = self.get_clock().now().to_msg()

        # -------------------------
        # Scalar fields
        # -------------------------
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        # -------------------------
        # Arrays (ranges / intensities)
        # -------------------------
        # Convert to list to ensure a real copy (avoid referencing same array buffer)
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        # Publish the restamped scan
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanRestamper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
