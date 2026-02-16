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
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

from rosgraph_msgs.msg import Clock as ClockMsg
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy



def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MujocoToGazeboBridge(Node):
    def __init__(self):
        super().__init__("mujoco_to_gazebo_bridge")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value

        self.mujoco_pose_topic = self.declare_parameter(
            "mujoco_pose_topic", "/mujoco/base_pose"
        ).value

        self.odom_topic = self.declare_parameter(
            "odom_topic", "/odom"
        ).value

        self.publish_map_odom_identity = self.declare_parameter(
            "publish_map_odom_identity", True
        ).value

        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # (on peut garder sys_clock, mais il ne sert plus à stamper)
        self.sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        self.last_pose_msg = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pose = False

        # prev: (t_sec, x, y, yaw) basé sur /clock uniquement
        self.prev = None

        self.bootstrap_sent = False
        self.map_odom_owned_by_slam = False

        self.create_subscription(PoseStamped, self.mujoco_pose_topic, self.pose_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Timer: IMPORTANT => ce node DOIT être lancé avec use_sim_time:=false (Option A)
        period = 1.0 / max(1e-6, self.rate_hz)
        self.create_timer(period, self.publish)

        self.get_logger().info(
            f"Bridge ready: {self.mujoco_pose_topic} → TF({self.odom_frame}->{self.base_frame}) + {self.odom_topic}"
        )
        self.get_logger().info(
            "Option A: run this node with use_sim_time=false. Stamp=0 until /clock exists, then stamp=/clock."
        )

        # /clock monitoring (consume only)
        self.last_clock: Optional[TimeMsg] = None
        clock_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(ClockMsg, "/clock", self.clock_cb, clock_qos)


    def pose_cb(self, msg: PoseStamped):
        self.last_pose_msg = msg
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.yaw = norm_angle(quat_to_yaw(msg.pose.orientation))
        self.have_pose = True

        self.get_logger().info(
            f"RX base_pose x={self.x:.3f} y={self.y:.3f}",
            throttle_duration_sec=1.0
        )

    def clock_cb(self, msg: ClockMsg):
        self.last_clock = msg.clock

    @staticmethod
    def _is_nonzero_time(t: Optional[TimeMsg]) -> bool:
        return (t is not None) and (t.sec != 0 or t.nanosec != 0)

    def _select_stamp(self) -> TimeMsg:
        if self._is_nonzero_time(self.last_clock):
            return self.last_clock  # type: ignore[return-value]
        z = TimeMsg()
        z.sec = 0
        z.nanosec = 0
        return z

    def _maybe_publish_map_odom(self):
        if self.map_odom_owned_by_slam:
            return

        try:
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            self.map_odom_owned_by_slam = True
            self.get_logger().info(
                "SLAM detected: map->odom is now owned by SLAM. Bridge will not publish bootstrap."
            )
            return
        except Exception:
            pass

        if not self.bootstrap_sent:
            t = TransformStamped()
            t.header.stamp = self._select_stamp()   # ✅ Option A
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0

            self.static_tf_broadcaster.sendTransform(t)
            self.bootstrap_sent = True
            self.get_logger().info("Bootstrap TF map->odom (identity) published (static).")

    def publish(self):
        # ✅ Option A: stamp comes from /clock if available, else 0
        stamp = self._select_stamp()
        sim_active = self._is_nonzero_time(self.last_clock)

        # Pose selection
        msg = self.last_pose_msg
        if msg is not None:
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            yaw = norm_angle(quat_to_yaw(msg.pose.orientation))
            self.x, self.y, self.yaw = x, y, yaw
            self.have_pose = True
        else:
            x, y, yaw = self.x, self.y, self.yaw

        qx, qy, qz, qw = yaw_to_quat(yaw)

        # Velocity estimation only when /clock is active
        vx = vy = wz = 0.0
        if sim_active and self.have_pose and (self.last_clock is not None):
            t_sec = float(self.last_clock.sec) + 1e-9 * float(self.last_clock.nanosec)

            if self.prev is not None:
                t0, x0, y0, yaw0 = self.prev
                dt = max(1e-6, t_sec - t0)
                vx = (x - x0) / dt
                vy = (y - y0) / dt
                wz = norm_angle(yaw - yaw0) / dt

            self.prev = (t_sec, x, y, yaw)
        else:
            self.prev = None

        # TF: odom -> base_frame
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

        # Odometry
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

        if self.publish_map_odom_identity:
            self._maybe_publish_map_odom()


def main():
    rclpy.init()
    node = MujocoToGazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
