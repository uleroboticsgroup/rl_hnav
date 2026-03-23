#!/usr/bin/env python3
"""
Diagnostic: compare /mujoco/joint_states (what MuJoCo wants)
vs /joint_states (what Gazebo actually has) for the ankle/foot joints.
Also check if the plugin is logging any kinematic mode messages.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

FOOT_JOINTS = [
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'left_hip_pitch_joint',
    'left_knee_joint',
    'right_hip_pitch_joint',
    'right_knee_joint',
]

class JointDiag(Node):
    def __init__(self):
        super().__init__('joint_diag')
        self.mj = {}
        self.gz = {}
        self.mj_ts = None
        self.gz_ts = None
        self.samples = 0
        
        self.create_subscription(JointState, '/mujoco/joint_states', self.mj_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.gz_cb, 10)
        self.timer = self.create_timer(2.0, self.report)
        self.get_logger().info('Waiting for joint data...')
        
    def mj_cb(self, msg):
        self.mj_ts = time.time()
        for name, pos in zip(msg.name, msg.position):
            self.mj[name] = pos
            
    def gz_cb(self, msg):
        self.gz_ts = time.time()
        for name, pos in zip(msg.name, msg.position):
            self.gz[name] = pos
    
    def report(self):
        self.samples += 1
        
        # Check if we receive both
        if not self.mj:
            self.get_logger().warn('No /mujoco/joint_states received yet')
            return
        if not self.gz:
            self.get_logger().warn('No /joint_states received yet — is robot_state_publisher publishing?')
            # Let's report what MuJoCo sends
            self.get_logger().info(f'MuJoCo has {len(self.mj)} joints')
            return
        
        self.get_logger().info(f'--- Sample {self.samples} ---')
        self.get_logger().info(f'MuJoCo: {len(self.mj)} joints | Gazebo: {len(self.gz)} joints')
        
        # Show foot joints comparison
        header = f'{"Joint":35s} {"MuJoCo":>10s} {"Gazebo":>10s} {"Error":>10s}'
        self.get_logger().info(header)
        
        for name in FOOT_JOINTS:
            mj_val = self.mj.get(name, None)
            gz_val = self.gz.get(name, None)
            if mj_val is not None and gz_val is not None:
                err = abs(mj_val - gz_val)
                err_str = f'{err:.6f}'
                if err > 0.01:
                    err_str += ' *** DRIFT'
                self.get_logger().info(
                    f'{name:35s} {mj_val:10.4f} {gz_val:10.4f} {err_str}')
            elif mj_val is not None:
                self.get_logger().info(f'{name:35s} {mj_val:10.4f}   N/A in Gazebo')
            elif gz_val is not None:
                self.get_logger().info(f'{name:35s}   N/A in MJ {gz_val:10.4f}')
        
        # Also check for joints in Gazebo NOT in MuJoCo
        gz_only = set(self.gz.keys()) - set(self.mj.keys())
        if gz_only:
            self.get_logger().warn(f'Joints in Gazebo BUT NOT in MuJoCo: {gz_only}')
        
        mj_only = set(self.mj.keys()) - set(self.gz.keys())
        if mj_only:
            self.get_logger().warn(f'Joints in MuJoCo BUT NOT in Gazebo: {mj_only}')

def main():
    rclpy.init()
    node = JointDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
