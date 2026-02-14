#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


@dataclass
class PlanarState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class HumanoidNavBridge(Node):
    """
    Minimal bridge to make Nav2 happy:
      - Subscribe: /cmd_vel (Twist)
      - Publish:   /odom (Odometry)
      - Publish TF: odom -> base_footprint   (planar)

    Backend strategy:
      - "dummy": integrates cmd_vel into pose (validate Nav2/TF pipeline)
      - later: swap update_state_from_backend() to read pose from MuJoCo / SDK2.
    """

    def __init__(self):
        super().__init__("humanoid_nav_bridge")

        # -------- Parameters --------
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "odom")  # relative by default

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("publish_basefootprint_to_baselink", True)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("backend", "dummy")  # dummy | mujoco | unitree_sdk2

        # Safety
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("max_vx", 0.8)
        self.declare_parameter("max_vy", 0.4)
        self.declare_parameter("max_wz", 1.2)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.base_link_frame = str(self.get_parameter("base_link_frame").value)
        self.publish_bf_to_bl = bool(self.get_parameter("publish_basefootprint_to_baselink").value)

        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.backend = str(self.get_parameter("backend").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        # -------- State --------
        self.state = PlanarState()
        self._last_cmd_time = self.get_clock().now()

        # -------- ROS I/O --------
        # IMPORTANT:
        # - If cmd_vel_topic is absolute (/cmd_vel), do NOT prefix with namespace.
        # - If odom_topic is relative (odom), ROS2 will namespace it automatically.
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        if self.publish_bf_to_bl:
            self.publish_static_basefootprint_to_baselink()

        # Timer loop
        period = 1.0 / max(1e-3, self.rate_hz)
        self._timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"[humanoid_nav_bridge] backend='{self.backend}' "
            f"sub={self.cmd_vel_topic} pub={self.odom_topic} TF: {self.odom_frame}->{self.base_frame} @ {self.rate_hz:.1f}Hz"
        )

        # --- Normalisation vers backend [-1..1] ---
        self.declare_parameter("use_backend_normalized_cmd", True)

        # ramp limiter (sur commandes normalisées)
        self.declare_parameter("max_dvx_norm_per_s", 1.2)
        self.declare_parameter("max_dvy_norm_per_s", 1.2)
        self.declare_parameter("max_dwz_norm_per_s", 2.5)

        self.use_backend_normalized_cmd = bool(self.get_parameter("use_backend_normalized_cmd").value)
        self.max_dvx_norm_per_s = float(self.get_parameter("max_dvx_norm_per_s").value)
        self.max_dvy_norm_per_s = float(self.get_parameter("max_dvy_norm_per_s").value)
        self.max_dwz_norm_per_s = float(self.get_parameter("max_dwz_norm_per_s").value)

        # état commande normalisée courante (après ramp)
        self._cmd_norm_target = [0.0, 0.0, 0.0]   # vx, vy, wz
        self._cmd_norm_current = [0.0, 0.0, 0.0]


    # ---------- Helpers ----------
    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def yaw_to_quat(yaw: float):
        """Quaternion for roll=0, pitch=0, yaw=yaw."""
        half = 0.5 * yaw
        qx = 0.0
        qy = 0.0
        qz = math.sin(half)
        qw = math.cos(half)
        return qx, qy, qz, qw

    def _ramp(self, cur, tgt, max_delta):
        if tgt > cur:
            return min(tgt, cur + max_delta)
        return max(tgt, cur - max_delta)

    def step_backend_command(self, dt: float):
        # deadman: si pas de cmd récente -> target=0
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._cmd_norm_target = [0.0, 0.0, 0.0]

        # ramp limiter sur normalisé
        max_dvx = self.max_dvx_norm_per_s * dt
        max_dvy = self.max_dvy_norm_per_s * dt
        max_dwz = self.max_dwz_norm_per_s * dt

        self._cmd_norm_current[0] = self._ramp(self._cmd_norm_current[0], self._cmd_norm_target[0], max_dvx)
        self._cmd_norm_current[1] = self._ramp(self._cmd_norm_current[1], self._cmd_norm_target[1], max_dvy)
        self._cmd_norm_current[2] = self._ramp(self._cmd_norm_current[2], self._cmd_norm_target[2], max_dwz)

        # Hook backend (à compléter plus tard)
        self.send_command_to_backend(self._cmd_norm_current[0],
                                     self._cmd_norm_current[1],
                                     self._cmd_norm_current[2])

    def send_command_to_backend(self, vx_n: float, vy_n: float, wz_n: float):
        if self.backend == "dummy":
            # on log (debug) uniquement
            return

        if self.backend == "mujoco":
            # TODO: publier vers un topic mujoco / ou appeler une API si tu as un wrapper
            return

        if self.backend == "unitree_sdk2":
            # TODO: appeler le SDK (High Level)
            return

    def publish_static_basefootprint_to_baselink(self):
        # By default: identity transform.
        # If your URDF defines base_link above the ground with z offset, you can adjust here.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"[humanoid_nav_bridge] Static TF published: {self.base_frame} -> {self.base_link_frame}")

    # ---------- Callbacks ----------
    def on_cmd_vel_(self, msg: Twist):
        vx = self.clamp(msg.linear.x, -self.max_vx, self.max_vx)
        vy = self.clamp(msg.linear.y, -self.max_vy, self.max_vy)
        wz = self.clamp(msg.angular.z, -self.max_wz, self.max_wz)

        self.state.vx = vx
        self.state.vy = vy
        self.state.wz = wz
        self._last_cmd_time = self.get_clock().now()

        # Later: send_command_to_backend(vx, vy, wz)
    
    def on_cmd_vel(self, msg: Twist):
        # clamp physique
        vx = self.clamp(msg.linear.x, -self.max_vx, self.max_vx)
        vy = self.clamp(msg.linear.y, -self.max_vy, self.max_vy)
        wz = self.clamp(msg.angular.z, -self.max_wz, self.max_wz)

        self.state.vx = vx
        self.state.vy = vy
        self.state.wz = wz
        self._last_cmd_time = self.get_clock().now()

        # normalisation [-1..1] pour backend
        vx_n = vx / self.max_vx if self.max_vx > 1e-6 else 0.0
        vy_n = vy / self.max_vy if self.max_vy > 1e-6 else 0.0
        wz_n = wz / self.max_wz if self.max_wz > 1e-6 else 0.0
        vx_n = self.clamp(vx_n, -1.0, 1.0)
        vy_n = self.clamp(vy_n, -1.0, 1.0)
        wz_n = self.clamp(wz_n, -1.0, 1.0)

        self._cmd_norm_target = [vx_n, vy_n, wz_n]

        # Later: send_command_to_backend(...) depuis on_timer (après ramp), pas ici


    def update_state_from_backend(self, dt: float):
        """
        For now: dummy integration.
        Later: replace with pose read from MuJoCo/SDK2, e.g. pelvis/world pose -> planar projection.
        """
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.wz = 0.0

        # Integrate base-local twist into world
        cy = math.cos(self.state.yaw)
        sy = math.sin(self.state.yaw)

        wx = cy * self.state.vx - sy * self.state.vy
        wy = sy * self.state.vx + cy * self.state.vy

        self.state.x += wx * dt
        self.state.y += wy * dt
        self.state.yaw += self.state.wz * dt

        # normalize yaw
        if self.state.yaw > math.pi or self.state.yaw < -math.pi:
            self.state.yaw = math.atan2(math.sin(self.state.yaw), math.cos(self.state.yaw))

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(self.state.x)
        t.transform.translation.y = float(self.state.y)
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quat(float(self.state.yaw))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame

        o.pose.pose.position.x = float(self.state.x)
        o.pose.pose.position.y = float(self.state.y)
        o.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quat(float(self.state.yaw))
        o.pose.pose.orientation.x = qx
        o.pose.pose.orientation.y = qy
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw

        o.twist.twist.linear.x = float(self.state.vx)
        o.twist.twist.linear.y = float(self.state.vy)
        o.twist.twist.angular.z = float(self.state.wz)

        # Minimal covariances (Nav2 likes non-zero)
        for i in range(36):
            o.pose.covariance[i] = 0.0
            o.twist.covariance[i] = 0.0
        o.pose.covariance[0] = 0.05   # x
        o.pose.covariance[7] = 0.05   # y
        o.pose.covariance[35] = 0.1   # yaw
        o.twist.covariance[0] = 0.1
        o.twist.covariance[7] = 0.1
        o.twist.covariance[35] = 0.2

        self.pub_odom.publish(o)

    def on_timer(self):
        now = self.get_clock().now()
        if not hasattr(self, "_prev_time"):
            self._prev_time = now
            return

        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(1e-3, self.rate_hz)
        
        self.step_backend_command(dt)
        # État: dummy integration tant que backend n’est pas branché
        self.update_state_from_backend(dt)
        stamp = now.to_msg()
        self.publish_tf(stamp)
        self.publish_odom(stamp)

def main():
    rclpy.init()
    node = HumanoidNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
