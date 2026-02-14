from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_node = Node(
        package="humanoid_nav_bridge",
        executable="bridge_node",
        name="humanoid_nav_bridge",
        namespace="g1_bridge",
        output="screen",
        parameters=[{
            
            "use_sim_time": True,
            # -------- Topics --------
            # Nav2 publishes cmd_vel globally; we subscribe globally.
            "cmd_vel_topic": "/cmd_vel",

            # Publish odom under the bridge namespace to avoid collisions
            "odom_topic": "odom",

            # -------- Frames --------
            "odom_frame": "odom",
            "base_frame": "base_footprint",     # Nav2 base
            "base_link_frame": "base_link",     # robot base link

            # Publish static TF base_footprint -> base_link
            "publish_basefootprint_to_baselink": True,

            # -------- Timing --------
            "publish_rate_hz": 50.0,

            # -------- Backend --------
            # dummy | mujoco | unitree_sdk2
            "backend": "dummy",

            # -------- Safety / Limits --------
            "cmd_timeout_sec": 0.5,
            "max_vx": 0.8,
            "max_vy": 0.4,
            "max_wz": 1.2,
        }],
    )

    return LaunchDescription([bridge_node])
