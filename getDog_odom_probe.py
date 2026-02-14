#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

class OdomProbe(Node):
    def __init__(self):
        super().__init__("getDog_odom_probe")
        topic = self.declare_parameter("topic", "/dog_odom").value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Odometry, topic, self.cb, qos)
        self.get_logger().info(f"Waiting for one Odometry on {topic} ...")

    def cb(self, msg: Odometry):
        self.get_logger().info(f"header.frame_id='{msg.header.frame_id}', child_frame_id='{msg.child_frame_id}'")
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(OdomProbe())

if __name__ == "__main__":
    main()
#RUN
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# python3 getDog_odom_probe.py