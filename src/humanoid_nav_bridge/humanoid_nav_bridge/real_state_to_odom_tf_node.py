#!/usr/bin/env python3
"""
real_state_to_odom_tf.py

Production-ready ROS 2 (Humble) bridge for real robots where:
- The robot publishes Odometry (e.g., /dog_odom),
- But Nav2 / slam_toolbox still require a consistent TF tree and a standard /odom topic.

This node:
  1) Subscribes to an input nav_msgs/Odometry topic (robot/native odom).
  2) Republishes a standardized /odom (nav_msgs/Odometry) with configured frame_ids.
  3) Publishes TF transform: odom_frame -> base_frame (dynamic).
  4) Optionally bootstraps map_frame -> odom_frame (identity) until SLAM publishes it.

Why it exists:
  Nav2 + slam_toolbox expect: map -> odom -> robot_base_frame (TF)
  Many real robot stacks publish odometry but do not publish TF in this form.

Typical setup:
  - Input:  /dog_odom   (header.frame_id="odom", child_frame_id="robot_center")
  - Output: /odom       (header.frame_id="odom", child_frame_id="robot_center" or "base_footprint")
  - TF:     odom -> robot_center  (or odom -> base_footprint if you force it)

Author: You (G1 / Unitree navigation integration)
License: your choice
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class RealOdomTfBridge(Node):
    """
    Bridge for real robot odometry into Nav2-friendly /odom + TF(odom->base).

    Key parameters
    --------------
    input_odom_topic (str):
        Input Odometry topic from the robot (default: /dog_odom)

    output_odom_topic (str):
        Output standardized Odometry topic for Nav2 (default: /odom)

    map_frame (str), odom_frame (str), base_frame (str):
        Frame ids to enforce on the output TF and /odom.

    force_child_frame (bool):
        If True, we ignore incoming msg.child_frame_id and enforce base_frame.
        This is recommended to avoid inconsistent TF trees.

    publish_map_odom_identity (bool):
        If True, publish a static identity TF map->odom at startup, BUT stop if SLAM publishes map->odom.

    rate_hz (float):
        Publish rate for TF and /odom (default: 50Hz)

    Notes
    -----
    - This node does not create base_link, URDF, or robot_state_publisher.
      It only guarantees the minimal TF chain required by Nav2/SLAM.
    """

    def __init__(self) -> None:
        super().__init__("real_odom_tf_bridge")

        # ----------------------------
        # Parameters
        # ----------------------------
        self.input_odom_topic = self.declare_parameter("input_odom_topic", "/dog_odom").value
        self.output_odom_topic = self.declare_parameter("output_odom_topic", "/odom").value

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "robot_center").value

        self.force_child_frame = self.declare_parameter("force_child_frame", True).value

        self.publish_map_odom_identity = self.declare_parameter("publish_map_odom_identity", False).value
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)

        # If True, publish TF even when no new odom has arrived yet (not recommended).
        self.publish_without_input = self.declare_parameter("publish_without_input", False).value

        # ----------------------------
        # QoS
        # ----------------------------
        # Many robot stacks publish odometry RELIABLE. Keep subscription flexible.
        # If you ever see "no messages" but topics exist, try BEST_EFFORT here.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ----------------------------
        # TF utilities
        # ----------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # ----------------------------
        # State
        # ----------------------------
        self._last_odom: Optional[Odometry] = None
        self._bootstrap_sent = False
        self._map_odom_owned_by_slam = False

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self._odom_pub = self.create_publisher(Odometry, self.output_odom_topic, pub_qos)
        self._odom_sub = self.create_subscription(Odometry, self.input_odom_topic, self._odom_cb, sub_qos)

        period = 1.0 / max(1e-6, self.rate_hz)
        self._timer = self.create_timer(period, self._on_timer)

        # Startup log
        self.get_logger().info(
            "RealOdomTfBridge started:\n"
            f"  input_odom_topic   = {self.input_odom_topic}\n"
            f"  output_odom_topic  = {self.output_odom_topic}\n"
            f"  frames: map={self.map_frame}, odom={self.odom_frame}, base={self.base_frame}\n"
            f"  force_child_frame  = {self.force_child_frame}\n"
            f"  publish_map_odom_identity = {self.publish_map_odom_identity}\n"
            f"  rate_hz            = {self.rate_hz}"
        )

    # ----------------------------
    # Callbacks
    # ----------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        """Store the latest Odometry message (we publish at a controlled rate in the timer)."""
        self._last_odom = msg

    # ----------------------------
    # Helpers
    # ----------------------------
    def _now_stamp(self):
        """
        Prefer node clock time. On real robot, use_sim_time should be false.
        If time is zero (rare), fall back to message stamp later.
        """
        now = self.get_clock().now()
        return now.to_msg(), now.nanoseconds

    def _maybe_bootstrap_map_odom(self) -> None:
        """
        Optional: publish static identity map->odom until SLAM publishes it.
        This prevents early TF lookup failures during startup.

        Important:
        - slam_toolbox normally owns map->odom. If it is active, we stop bootstrapping.
        """
        if not self.publish_map_odom_identity or self._map_odom_owned_by_slam:
            return

        # Detect whether SLAM is already publishing map->odom
        try:
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            self._map_odom_owned_by_slam = True
            if self._bootstrap_sent:
                self.get_logger().info("SLAM detected: map->odom now owned by SLAM. Bridge stops bootstrapping.")
            return
        except Exception:
            pass  # SLAM not ready yet

        if not self._bootstrap_sent:
            t = TransformStamped()
            t.header.stamp = rclpy.time.Time().to_msg()  # static
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame
            t.transform.rotation.w = 1.0  # identity
            self.static_tf_broadcaster.sendTransform(t)
            self._bootstrap_sent = True
            self.get_logger().info("Bootstrap TF published: map->odom (identity).")

    # ----------------------------
    # Main publishing loop
    # ----------------------------
    def _on_timer(self) -> None:
        """
        Publish:
          - TF(odom_frame -> base_frame)
          - /odom standardized message

        The node publishes only when it has received input odometry,
        unless publish_without_input=True.
        """
        msg = self._last_odom
        if msg is None and not self.publish_without_input:
            self._maybe_bootstrap_map_odom()
            return

        # Choose timestamp:
        # - Prefer node clock time (monotonic & consistent for TF consumers).
        # - If node time is zero (rare), fallback to incoming message stamp.
        stamp, now_ns = self._now_stamp()
        if now_ns == 0:
            stamp = msg.header.stamp if msg is not None else rclpy.time.Time().to_msg()

        # Extract pose/twist from incoming odom (if missing input, publish zeros)
        if msg is not None:
            pose = msg.pose
            twist = msg.twist
            src_child = (msg.child_frame_id or "").strip()
        else:
            # Safe defaults
            pose = Odometry().pose
            twist = Odometry().twist
            src_child = ""

        child_frame = self.base_frame if self.force_child_frame else (src_child or self.base_frame)

        # ----------------------------
        # Publish TF: odom_frame -> child_frame
        # ----------------------------
        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = self.odom_frame
        tfm.child_frame_id = child_frame

        tfm.transform.translation.x = float(pose.pose.position.x)
        tfm.transform.translation.y = float(pose.pose.position.y)
        tfm.transform.translation.z = float(pose.pose.position.z)

        tfm.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(tfm)

        # ----------------------------
        # Republish standardized /odom
        # ----------------------------
        odom_out = Odometry()
        odom_out.header.stamp = stamp
        odom_out.header.frame_id = self.odom_frame
        odom_out.child_frame_id = child_frame
        odom_out.pose = pose
        odom_out.twist = twist
        self._odom_pub.publish(odom_out)

        # Throttled debug
        self.get_logger().info(
            f"Publishing TF {self.odom_frame}->{child_frame} and {self.output_odom_topic} "
            f"(x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f})",
            throttle_duration_sec=2.0,
        )

        # Bootstrap map->odom if requested
        self._maybe_bootstrap_map_odom()


def main() -> None:
    rclpy.init()
    node = RealOdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
