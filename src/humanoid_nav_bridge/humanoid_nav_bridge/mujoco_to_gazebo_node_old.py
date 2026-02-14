#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


# ---------------------------
# Utils
# ---------------------------
def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class MujocoToGazeboBridge(Node):
    def __init__(self):
        super().__init__("mujoco_to_gazebo_bridge")

        self.odom_frame = "odom"
        self.base_frame = "base_footprint"

        self.create_subscription(
            PoseStamped,
            "/mujoco/base_pose",
            self.pose_cb,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.prev = None
        self.create_timer(0.02, self.publish)  # 50 Hz
