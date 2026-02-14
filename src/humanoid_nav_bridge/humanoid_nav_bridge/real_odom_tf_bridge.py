#!/usr/bin/env python3
"""
real_odom_tf_bridge.py

Production-ready ROS 2 node bridging a robot Odometry topic to:
  - a standardized /odom topic (Nav2-compatible)
  - TF transform odom_frame -> base_frame

Why this node exists
--------------------
Many real robots publish nav_msgs/Odometry (e.g. /dog_odom) but do not publish
(or publish inconsistently) the TF tree required by Nav2 / slam_toolbox:

    map -> odom -> robot_base_frame

This node provides the missing dynamic TF link (odom -> base_frame) and a clean
/odom stream with consistent frame_ids.

Key features (production-grade)
-------------------------------
- Optional "zero_at_start": subtract initial pose so robot starts at (0,0,0).
  This prevents Nav2 costmap "out of bounds" issues when odom starts far from origin.
- Decoupled publishing using timers:
    * TF published at tf_rate_hz
    * /odom published at odom_rate_hz
  This stabilizes TF caches and message_filters.
- Robust timestamp policy:
    * use_node_time=True: stamp with node clock now() (recommended on real robot)
    * use_node_time=False: reuse incoming msg.header.stamp
- Child frame control:
    * force_child_frame=True: always publish base_frame (recommended)
- Safety monitoring:
    * warning if odom stream is stale (timeout_sec)

Typical Unitree case
--------------------
/dog_odom: header.frame_id="odom", child_frame_id="robot_center"

Recommended defaults:
  odom_frame="odom"
  base_frame="robot_center"

Author: Jean Chrysostome Mayoko Biong
License: Apache-2.0
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def _make_qos(reliable: bool, depth: int) -> QoSProfile:
    """Helper to build a QoSProfile compatible with many real robot DDS setups."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class RealOdomTfBridge(Node):
    """
    Subscribe to an input Odometry topic and publish:
      1) TF: odom_frame -> base_frame
      2) /odom standardized (same frames / stamp policy)

    Parameters
    ----------
    input_odom_topic (str):   Input odom topic (default: /dog_odom)
    output_odom_topic (str):  Output /odom topic (default: /odom)

    odom_frame (str): TF parent frame (default: odom)
    base_frame (str): TF child frame / Nav2 base frame (default: robot_center)

    force_child_frame (bool): If True, ignore msg.child_frame_id and always use base_frame.
    use_node_time (bool):     If True, stamp outgoing messages with node clock now().

    zero_at_start (bool):     If True, subtract initial pose (x,y,z + yaw) from all outgoing poses.
                              Helps to start Nav2/SLAM at origin and avoid global_costmap "out of bounds".

    publish_tf (bool):        Enable TF publishing.
    publish_odom (bool):      Enable /odom publishing.

    tf_rate_hz (float):       TF publish frequency (default: 50).
    odom_rate_hz (float):     /odom publish frequency (default: 50).

    timeout_sec (float):      If no input odom is received for this duration, log a warning.

    QoS
    ---
    - Subscriber: RELIABLE by default (common on Unitree bare DDS).
    - Publisher: RELIABLE by default to match Nav2 expectations.
      You can flip to BEST_EFFORT if needed.
    """

    def __init__(self) -> None:
        super().__init__("real_odom_tf_bridge")

        # ----------------------------
        # Parameters (topics + frames)
        # ----------------------------
        self.input_odom_topic = self.declare_parameter("input_odom_topic", "/dog_odom").value
        self.output_odom_topic = self.declare_parameter("output_odom_topic", "/odom").value

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "robot_center").value

        # Behavior toggles
        self.force_child_frame = bool(self.declare_parameter("force_child_frame", True).value)
        self.use_node_time = bool(self.declare_parameter("use_node_time", True).value)
        self.zero_at_start = bool(self.declare_parameter("zero_at_start", False).value)

        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.publish_odom = bool(self.declare_parameter("publish_odom", True).value)

        # Rates + timeout
        self.tf_rate_hz = float(self.declare_parameter("tf_rate_hz", 50.0).value)
        self.odom_rate_hz = float(self.declare_parameter("odom_rate_hz", 50.0).value)
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 1.0).value)

        # QoS knobs
        self.sub_reliable = bool(self.declare_parameter("sub_reliable", True).value)
        self.pub_reliable = bool(self.declare_parameter("pub_reliable", True).value)
        self.qos_depth = int(self.declare_parameter("qos_depth", 10).value)

        sub_qos = _make_qos(self.sub_reliable, self.qos_depth)
        pub_qos = _make_qos(self.pub_reliable, self.qos_depth)

        # ----------------------------
        # Publishers / TF broadcaster
        # ----------------------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, pub_qos)

        # ----------------------------
        # Subscription
        # ----------------------------
        self._last_msg: Optional[Odometry] = None
        self._last_rx_time: Optional[Time] = None

        self.sub = self.create_subscription(Odometry, self.input_odom_topic, self._cb, sub_qos)

        # ----------------------------
        # Zero-at-start offset (computed once from first msg)
        # ----------------------------
        self._zero_ready = False
        self._zero_xyz_yaw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

        # ----------------------------
        # Timers (stable publishing)
        # ----------------------------
        if self.publish_tf and self.tf_rate_hz > 0.0:
            self._tf_timer = self.create_timer(1.0 / self.tf_rate_hz, self._publish_tf_tick)
        else:
            self._tf_timer = None

        if self.publish_odom and self.odom_rate_hz > 0.0:
            self._odom_timer = self.create_timer(1.0 / self.odom_rate_hz, self._publish_odom_tick)
        else:
            self._odom_timer = None

        self._watchdog_timer = self.create_timer(0.5, self._watchdog_tick)

        # ----------------------------
        # Startup log
        # ----------------------------
        self.get_logger().info(
            "RealOdomTfBridge started with:\n"
            f"  input_odom_topic  = {self.input_odom_topic}\n"
            f"  output_odom_topic = {self.output_odom_topic}\n"
            f"  odom_frame        = {self.odom_frame}\n"
            f"  base_frame        = {self.base_frame}\n"
            f"  force_child_frame = {self.force_child_frame}\n"
            f"  use_node_time     = {self.use_node_time}\n"
            f"  zero_at_start     = {self.zero_at_start}\n"
            f"  publish_tf        = {self.publish_tf} @ {self.tf_rate_hz} Hz\n"
            f"  publish_odom      = {self.publish_odom} @ {self.odom_rate_hz} Hz\n"
            f"  qos: sub_reliable={self.sub_reliable} pub_reliable={self.pub_reliable} depth={self.qos_depth}\n"
            f"  timeout_sec       = {self.timeout_sec}"
        )

    # ----------------------------
    # Callbacks
    # ----------------------------
    def _cb(self, msg: Odometry) -> None:
        """Store the latest odometry message. Publishing is done by timers."""
        self._last_msg = msg
        self._last_rx_time = self.get_clock().now()

        # Initialize zero offset once (first valid message)
        if self.zero_at_start and not self._zero_ready:
            x0 = float(msg.pose.pose.position.x)
            y0 = float(msg.pose.pose.position.y)
            z0 = float(msg.pose.pose.position.z)
            yaw0 = self._yaw_from_quat(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            self._zero_xyz_yaw = (x0, y0, z0, yaw0)
            self._zero_ready = True
            self.get_logger().info(
                f"zero_at_start: locked offset x0={x0:.3f}, y0={y0:.3f}, z0={z0:.3f}, yaw0={yaw0:.3f} rad"
            )

    def _watchdog_tick(self) -> None:
        """Warn if the incoming odom stream is stale."""
        if self._last_rx_time is None:
            self.get_logger().warn("No odom received yet...", throttle_duration_sec=2.0)
            return

        age = (self.get_clock().now() - self._last_rx_time).nanoseconds * 1e-9
        if age > self.timeout_sec:
            self.get_logger().warn(
                f"Odometry stream stale: last msg age = {age:.2f}s (timeout={self.timeout_sec:.2f}s). "
                "TF/odom publishing will keep last pose.",
                throttle_duration_sec=2.0,
            )

    # ----------------------------
    # Publishing ticks
    # ----------------------------
    def _publish_tf_tick(self) -> None:
        """Publish TF odom_frame -> base_frame from the last received odometry."""
        if self._last_msg is None:
            return

        msg = self._last_msg
        stamp = self._choose_stamp(msg)

        child = self._choose_child_frame(msg)

        x, y, z, qx, qy, qz, qw = self._extract_pose(msg)

        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = self.odom_frame
        tfm.child_frame_id = child
        tfm.transform.translation.x = x
        tfm.transform.translation.y = y
        tfm.transform.translation.z = z
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tfm)

        self.get_logger().info(
            f"Publishing TF {self.odom_frame}->{child}",
            throttle_duration_sec=2.0,
        )

    def _publish_odom_tick(self) -> None:
        """Republish standardized /odom from the last received odometry."""
        if self._last_msg is None:
            return

        msg = self._last_msg
        stamp = self._choose_stamp(msg)

        child = self._choose_child_frame(msg)

        x, y, z, qx, qy, qz, qw = self._extract_pose(msg)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = child

        # Pose
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = msg.pose.covariance

        # Twist
        out.twist = msg.twist  # ok in practice; can deep-copy if you want
        self.odom_pub.publish(out)

        self.get_logger().info(
            f"Publishing {self.output_odom_topic} (child_frame={child})",
            throttle_duration_sec=2.0,
        )

    # ----------------------------
    # Helpers
    # ----------------------------
    def _choose_stamp(self, msg: Odometry):
        """Choose outgoing timestamp based on policy."""
        if self.use_node_time:
            return self.get_clock().now().to_msg()
        return msg.header.stamp

    def _choose_child_frame(self, msg: Odometry) -> str:
        """Choose child_frame_id based on force_child_frame policy."""
        if self.force_child_frame:
            return self.base_frame

        incoming = (msg.child_frame_id or "").strip()
        return incoming if incoming else self.base_frame

    def _extract_pose(self, msg: Odometry) -> Tuple[float, float, float, float, float, float, float]:
        """
        Extract (x,y,z, qx,qy,qz,qw) from msg.pose, and apply zero_at_start if enabled.
        We apply translation offset, and yaw offset (optional but recommended).
        """
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)

        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)

        if not self.zero_at_start:
            return x, y, z, qx, qy, qz, qw

        # If zero not ready yet, do not modify (first few ms); publishing will stabilize after first msg.
        if not self._zero_ready:
            return x, y, z, qx, qy, qz, qw

        x0, y0, z0, yaw0 = self._zero_xyz_yaw

        # Translate by initial offset
        x -= x0
        y -= y0
        z -= z0

        # Remove initial yaw (keep roll/pitch untouched) by rotating around Z.
        # Convert quat -> yaw, subtract yaw0, rebuild a yaw-only correction quat and apply it.
        yaw = self._yaw_from_quat(qx, qy, qz, qw)
        yaw_corr = yaw - yaw0

        # Build corrected quat from the original roll/pitch + corrected yaw is complex without full RPY.
        # For 2D navigation, a yaw-only orientation is sufficient and often preferable.
        # We therefore publish yaw-only orientation (roll/pitch = 0), which is standard for planar Nav2/SLAM.
        qx2, qy2, qz2, qw2 = self._quat_from_yaw(yaw_corr)

        return x, y, z, qx2, qy2, qz2, qw2

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        """Return yaw from quaternion (Z rotation), using standard conversion."""
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
        """Build quaternion representing a pure yaw rotation."""
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))


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
