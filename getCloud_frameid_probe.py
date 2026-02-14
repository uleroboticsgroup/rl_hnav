#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

class Probe(Node):
    def __init__(self):
        super().__init__("getCloud_frameid_probe")
        topic = self.declare_parameter("topic", "/utlidar/cloud_livox_mid360").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(PointCloud2, topic, self.cb, qos)
        self.get_logger().info(f"Waiting for one PointCloud2 on {topic} ...")

    def cb(self, msg: PointCloud2):
        self.get_logger().info(f"frame_id='{msg.header.frame_id}' stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(Probe())

if __name__ == "__main__":
    main()

#RUN 
#echo $ROS_DOMAIN_ID
# python3 getCloud_frameid_probe.py --ros-args -p topic:=/utlidar/cloud_livox_mid360