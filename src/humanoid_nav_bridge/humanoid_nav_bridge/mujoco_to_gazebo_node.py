#!/usr/bin/env python3
"""
MuJoCo → Gazebo / Nav2 bridge (ROS 2, Python)

What this node does
-------------------
1) Subscribes to a MuJoCo pose topic: geometry_msgs/PoseStamped (e.g. /mujoco/base_pose)
2) Publishes:
   - TF:        odom -> base_footprint   (dynamic)
   - Odometry:  /odom                    (nav_msgs/Odometry)
3) Optionally publishes a bootstrap TF:
   - map -> odom (identity) as a STATIC transform
   - ONLY until SLAM starts publishing map->odom (then it stops "owning" it)

Why this version is robust
--------------------------
When use_sim_time:=true, /clock can be missing temporarily (gzserver instability).
If we timestamp TF using the ROS sim clock while /clock is missing, time is stuck at 0 and
Nav2 will fail with: "Invalid frame ID 'odom' ... frame does not exist".

To break the deadlock, this node:
- ALWAYS publishes odom->base_footprint (and /odom) even before the first MuJoCo pose,
  using a default pose (0,0,0). This ensures the 'odom' frame exists immediately.
- Uses SYSTEM_TIME as a fallback timestamp when sim time is not active (sim time == 0).
  Once /clock becomes available, it automatically switches to sim time timestamps.

Notes
-----
- This node publishes odom->base_footprint in ROS TF. You may still synchronize the Gazebo model pose
  separately if you want, but Nav2/SLAM depend on TF in ROS.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


# ---------------------------
# Small math utilities
# ---------------------------
def quat_to_yaw(q) -> float:
    """Convert quaternion (geometry_msgs/Quaternion) to yaw (rad)."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def yaw_to_quat(yaw: float):
    """Convert yaw (rad) to quaternion tuple (x,y,z,w) with roll=pitch=0."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def norm_angle(a: float) -> float:
    """Normalize angle to [-pi, +pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ---------------------------
# Bridge Node
# ---------------------------
class MujocoToGazeboBridge(Node):
    def __init__(self):
        super().__init__("mujoco_to_gazebo_bridge")

        # ----------------------------
        # Parameters: frames
        # ----------------------------
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value

        # ----------------------------
        # Parameters: topics
        # ----------------------------
        self.mujoco_pose_topic = self.declare_parameter(
            "mujoco_pose_topic", "/mujoco/base_pose"
        ).value

        self.odom_topic = self.declare_parameter(
            "odom_topic", "/odom"
        ).value

        # ----------------------------
        # Parameters: options
        # ----------------------------
        # If True: publish an identity map->odom static transform ONCE (bootstrap),
        # and automatically stop doing it when SLAM starts publishing map->odom.
        self.publish_map_odom_identity = self.declare_parameter(
            "publish_map_odom_identity", True
        ).value

        # Publishing frequency (TF + /odom)
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)

        # ----------------------------
        # TF buffer/listener (used to detect whether SLAM already publishes map->odom)
        # ----------------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------
        # Clocks
        # ----------------------------
        # get_clock() will follow use_sim_time parameter (sim clock)
        # sys_clock is always wall/system time (independent from /clock)
        self.sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # ----------------------------
        # State
        # ----------------------------
        self.last_pose_msg = None

        # Default pose: published even before receiving the first MuJoCo message
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pose = False  # becomes True after at least one MuJoCo pose received

        # Previous state for velocity estimation: (t_sec, x, y, yaw)
        self.prev = None

        # Bootstrap TF map->odom state
        self.bootstrap_sent = False
        self.map_odom_owned_by_slam = False

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.create_subscription(PoseStamped, self.mujoco_pose_topic, self.pose_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Periodic publish loop
        period = 1.0 / max(1e-6, self.rate_hz)
        self.create_timer(period, self.publish)

        self.get_logger().info(
            f"Bridge ready: {self.mujoco_pose_topic} → TF({self.odom_frame}->{self.base_frame}) + {self.odom_topic}"
        )
        self.get_logger().info(
            "Robust mode: publishes TF/odom even if /clock is missing (SYSTEM_TIME fallback)."
        )

    # ---------------------------
    # Callback: MuJoCo pose input
    # ---------------------------
    def pose_cb(self, msg: PoseStamped):
        """Store latest pose and update internal pose state."""
        self.last_pose_msg = msg
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.yaw = norm_angle(quat_to_yaw(msg.pose.orientation))
        self.have_pose = True

        self.get_logger().info(
            f"RX base_pose x={self.x:.3f} y={self.y:.3f}",
            throttle_duration_sec=1.0
        )

    # ---------------------------
    # Bootstrap map->odom identity (optional)
    # ---------------------------
    def _maybe_publish_map_odom(self):
        """
        Publish identity map->odom as a STATIC TF once (bootstrap),
        but stop if SLAM is detected publishing map->odom.
        """
        if self.map_odom_owned_by_slam:
            return

        # If SLAM already publishes map->odom, we should not publish our bootstrap
        try:
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            self.map_odom_owned_by_slam = True
            self.get_logger().info("SLAM detected: map->odom is now owned by SLAM. Bridge will not publish bootstrap.")
            return
        except Exception:
            pass  # SLAM not ready yet

        # Publish bootstrap only once
        if not self.bootstrap_sent:
            t = TransformStamped()

            # For static transform, TF2 can accept stamp=0. But we can still give a valid stamp:
            # use sim time if active else system time (to avoid any confusion).
            now_sim = self.get_clock().now()
            if now_sim.nanoseconds != 0:
                t.header.stamp = now_sim.to_msg()
            else:
                t.header.stamp = self.sys_clock.now().to_msg()

            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame

            # Identity transform
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0

            self.static_tf_broadcaster.sendTransform(t)
            self.bootstrap_sent = True
            self.get_logger().info("Bootstrap TF map->odom (identity) published by bridge (static).")

    # ---------------------------
    # Main periodic publisher
    # ---------------------------
    def publish(self):
        """
        Publish TF(odom->base_frame) and /odom.

        Key robustness feature:
        - If sim time is not active (no /clock): use SYSTEM_TIME for timestamps.
        - Always publish, even before first MuJoCo pose, using default pose.
        """
        # ----------------------------
        # Timestamp selection
        # ----------------------------
        now_sim = self.get_clock().now()
        sim_active = (now_sim.nanoseconds != 0)

        if sim_active:
            stamp = now_sim.to_msg()
            t_sec = now_sim.nanoseconds * 1e-9
        else:
            now_sys = self.sys_clock.now()
            stamp = now_sys.to_msg()
            t_sec = now_sys.nanoseconds * 1e-9

        # ----------------------------
        # Pose selection
        # ----------------------------
        # If we have a MuJoCo message: use it.
        # Otherwise: use the last known/default pose (0,0,0).
        msg = self.last_pose_msg
        if msg is not None:
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            yaw = norm_angle(quat_to_yaw(msg.pose.orientation))
            # Keep internal state updated
            self.x, self.y, self.yaw = x, y, yaw
            self.have_pose = True
        else:
            x, y, yaw = self.x, self.y, self.yaw

        qx, qy, qz, qw = yaw_to_quat(yaw)

        # ----------------------------
        # Velocity estimation
        # ----------------------------
        # Only compute meaningful velocities after we have at least one real pose.
        vx = vy = wz = 0.0
        if self.prev is not None and self.have_pose:
            t0, x0, y0, yaw0 = self.prev
            dt = max(1e-6, t_sec - t0)
            vx = (x - x0) / dt
            vy = (y - y0) / dt
            wz = norm_angle(yaw - yaw0) / dt

        self.prev = (t_sec, x, y, yaw)

        # ----------------------------
        # TF: odom -> base_frame
        # ----------------------------
        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = self.odom_frame
        tfm.child_frame_id = self.base_frame

        tfm.transform.translation.x = x
        tfm.transform.translation.y = y
        tfm.transform.translation.z = 0.0

        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tfm)

        self.get_logger().info(
            f"PUB TF {self.odom_frame}->{self.base_frame} x={x:.3f} y={y:.3f} sim_active={sim_active}",
            throttle_duration_sec=1.0
        )

        # ----------------------------
        # Odometry message
        # ----------------------------
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # ----------------------------
        # Bootstrap map->odom if requested
        # ----------------------------
        if self.publish_map_odom_identity:
            self._maybe_publish_map_odom()


# ---------------------------
# Main
# ---------------------------
def main():
    rclpy.init()
    node = MujocoToGazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Let launch/system handle shutdown gracefully
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
