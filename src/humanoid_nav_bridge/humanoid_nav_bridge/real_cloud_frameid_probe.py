#!/usr/bin/env python3
"""
cloud_frameid_probe.py

Production-ready "probe" node to quickly identify the header.frame_id of a PointCloud2
topic on a real robot (e.g., Unitree / Livox Mid-360).

Why this node exists
--------------------
When integrating SLAM/Nav2 on real robots, you must build a correct TF chain.
To do so, you need the *exact* sensor frame for the LiDAR stream:
  - PointCloud2.header.frame_id  (e.g., "livox_frame")

Some robots publish using "bare DDS apps" and QoS can be tricky. This node
tries RELIABLE first, and if no messages arrive within a timeout, retries
with BEST_EFFORT.

Typical usage
-------------
1) Default topic:
   ros2 run humanoid_nav_bridge cloud_frameid_probe

2) Custom topic:
   ros2 run humanoid_nav_bridge cloud_frameid_probe --ros-args -p cloud_topic:=/utlidar/cloud_livox_mid360

Outputs
-------
- Prints frame_id and timestamp for the first received message.
- Optionally exits after first message (default: True).

Author: (your name)
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


def make_qos(reliability: ReliabilityPolicy, depth: int = 1) -> QoSProfile:
    """Create a QoSProfile suitable for subscribing to real robot sensor topics."""
    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class CloudFrameIdProbe(Node):
    """
    Subscribe to a PointCloud2 topic and log the header.frame_id of the first message.

    Parameters
    ----------
    cloud_topic (str):
        PointCloud2 input topic.

    timeout_sec (float):
        How long to wait before falling back to another QoS or giving up.

    exit_after_first (bool):
        If True, shutdown after receiving the first valid message.
    """

    def __init__(self) -> None:
        super().__init__("cloud_frameid_probe")

        # ----------------------------
        # Parameters
        # ----------------------------
        self.cloud_topic = self.declare_parameter(
            "cloud_topic", "/utlidar/cloud_livox_mid360"
        ).value

        self.timeout_sec = float(self.declare_parameter("timeout_sec", 5.0).value)
        self.exit_after_first = bool(self.declare_parameter("exit_after_first", True).value)

        # ----------------------------
        # Internal state
        # ----------------------------
        self._got_msg = False
        self._start_time = time.time()
        self._sub = None  # created by _create_subscription()
        self._attempt = 0  # 0: RELIABLE, 1: BEST_EFFORT

        # Try RELIABLE first
        self._create_subscription(ReliabilityPolicy.RELIABLE)

        # Timer to enforce timeout / QoS fallback
        self._timer = self.create_timer(0.2, self._on_timer)

        self.get_logger().info(
            "CloudFrameIdProbe started:\n"
            f"  cloud_topic     = {self.cloud_topic}\n"
            f"  timeout_sec     = {self.timeout_sec}\n"
            f"  exit_after_first= {self.exit_after_first}\n"
            "  QoS attempt #1  = RELIABLE (will fallback to BEST_EFFORT if needed)"
        )

    def _create_subscription(self, reliability: ReliabilityPolicy) -> None:
        """
        Create (or recreate) the subscription with the specified reliability.
        We recreate the subscription to handle QoS mismatch issues.
        """
        if self._sub is not None:
            try:
                self.destroy_subscription(self._sub)
            except Exception:
                pass

        qos = make_qos(reliability=reliability, depth=1)
        self._sub = self.create_subscription(PointCloud2, self.cloud_topic, self._cb, qos)

        rel_str = "RELIABLE" if reliability == ReliabilityPolicy.RELIABLE else "BEST_EFFORT"
        self.get_logger().info(f"Subscribing to {self.cloud_topic} with QoS reliability={rel_str}")

    def _cb(self, msg: PointCloud2) -> None:
        """Handle first received PointCloud2 and report its frame_id."""
        if self._got_msg:
            return

        self._got_msg = True

        frame_id = (msg.header.frame_id or "").strip()
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)

        self.get_logger().info(
            "Received first PointCloud2:\n"
            f"  frame_id = '{frame_id}'\n"
            f"  stamp    = {sec}.{nsec:09d}"
        )

        if self.exit_after_first:
            # Clean shutdown: stop spinning and exit
            self.get_logger().info("exit_after_first=True -> shutting down.")
            rclpy.shutdown()

    def _on_timer(self) -> None:
        """
        Periodic check:
        - If no message arrives within timeout_sec, fallback from RELIABLE to BEST_EFFORT once.
        - If still no message after second attempt, give up and shutdown.
        """
        if self._got_msg:
            return

        elapsed = time.time() - self._start_time
        if elapsed < self.timeout_sec:
            return

        # Timeout reached
        if self._attempt == 0:
            # Retry with BEST_EFFORT
            self._attempt = 1
            self._start_time = time.time()
            self.get_logger().warn(
                f"No PointCloud2 received in {self.timeout_sec:.1f}s with RELIABLE. "
                "Retrying with BEST_EFFORT..."
            )
            self._create_subscription(ReliabilityPolicy.BEST_EFFORT)
            return

        # Second timeout -> give up
        self.get_logger().error(
            f"No PointCloud2 received after {self.timeout_sec:.1f}s (BEST_EFFORT). "
            "Check network, ROS_DOMAIN_ID, RMW implementation, or whether the sensor is running."
        )
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = CloudFrameIdProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
