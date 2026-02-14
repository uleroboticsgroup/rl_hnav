#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MujocoToGazeboModel(Node):
    """
    Fallback sync via gazebo_msgs/srv/SetModelState IF the service exists.
    If not, this node will keep warning and do nothing (use the Gazebo plugin instead).
    """

    def __init__(self):
        super().__init__("mujoco_to_gazebo_model")

        # Params
        self.mujoco_pose_topic = self.declare_parameter("mujoco_pose_topic", "/mujoco/base_pose").value
        self.model_name = self.declare_parameter("model_name", "g1").value
        self.reference_frame = self.declare_parameter("reference_frame", "world").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.use_twist = bool(self.declare_parameter("use_twist", True).value)
        self.lock_z = bool(self.declare_parameter("lock_z", True).value)
        self.fixed_z = float(self.declare_parameter("fixed_z", 0.92).value)

        # Only ONE parameter name, and a list of candidates
        default_candidates = ["/gazebo/set_model_state", "/set_model_state", "/gazebo/set_entity_state"]
        self.service_candidates = self.declare_parameter("service_candidates", default_candidates).value

        self.last_pose = None
        self.prev = None  # (t_sec, x, y, yaw)
        self._inflight = None

        self.create_subscription(PoseStamped, self.mujoco_pose_topic, self.pose_cb, 10)

        # Try to connect to a working service (SetModelState type)
        self.cli = None
        self.service_name = None
        for sname in self.service_candidates:
            cli = self.create_client(SetModelState, sname)
            if cli.wait_for_service(timeout_sec=0.2):
                self.cli = cli
                self.service_name = sname
                break

        if self.cli is None:
            self.get_logger().error(
                "No SetModelState service found. "
                "Use the Gazebo model plugin mujoco_pose_sync (recommended) or load the proper gazebo_ros state plugin."
            )
        else:
            self.get_logger().info(f"Using service: {self.service_name}")

        self.timer = self.create_timer(1.0 / max(1e-6, self.rate_hz), self.tick)

    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg

    def tick(self):
        if self.cli is None:
            return
        if self.last_pose is None:
            return

        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        if (self._inflight is not None) and (not self._inflight.done()):
            return

        msg = self.last_pose
        pose = msg.pose

        t_sec = float(now.nanoseconds) * 1e-9
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = self.fixed_z if self.lock_z else float(pose.position.z)

        yaw = norm_angle(quat_to_yaw(pose.orientation))

        vx = vy = wz = 0.0
        if self.use_twist and self.prev is not None:
            t0, x0, y0, yaw0 = self.prev
            dt = max(1e-6, t_sec - t0)
            vx = (x - x0) / dt
            vy = (y - y0) / dt
            wz = norm_angle(yaw - yaw0) / dt

        self.prev = (t_sec, x, y, yaw)

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = self.reference_frame

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation = pose.orientation

        state.twist.linear.x = float(vx)
        state.twist.linear.y = float(vy)
        state.twist.angular.z = float(wz)

        req = SetModelState.Request()
        req.model_state = state
        self._inflight = self.cli.call_async(req)


def main():
    rclpy.init()
    node = MujocoToGazeboModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
