from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic   = LaunchConfiguration("odom_topic")
    mujoco_pose_topic = LaunchConfiguration("mujoco_pose_topic")
    publish_map_odom_identity = LaunchConfiguration("publish_map_odom_identity")

    bridge = Node(
        package="humanoid_nav_bridge",
        executable="mtog_bridge",
        name="mujoco_to_gazebo_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "mujoco_pose_topic": mujoco_pose_topic,
            "odom_topic": odom_topic,
            "publish_map_odom_identity": publish_map_odom_identity,
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "rate_hz": 50.0,
        }],
    )

    model_sync = Node(
        package="humanoid_nav_bridge",
        executable="mujoco_to_gazebo_model",
        name="mujoco_to_gazebo_model",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "mujoco_pose_topic": mujoco_pose_topic,
            "model_name": "g1",
            "reference_frame": "world",
            "service_name": "/gazebo/set_model_state",
            "rate_hz": 30.0,          # ✅ mieux que 50
            "use_twist": True,
            "lock_z": True,
            "fixed_z": 0.75,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo/MuJoCo) clock"),
        DeclareLaunchArgument("mujoco_pose_topic", default_value="/mujoco/base_pose", description="PoseStamped published by MuJoCo"),
        DeclareLaunchArgument("odom_topic", default_value="/odom", description="Odometry topic consumed by Nav2"),
        DeclareLaunchArgument("publish_map_odom_identity", default_value="false", description="Publish static TF map->odom identity (debug SLAM)"),

        # ✅ Bridge TF/Odom démarre tôt
        TimerAction(period=1.0, actions=[bridge]),

        # ✅ Model sync démarre plus tard (après spawn Gazebo)
        #TimerAction(period=7.0, actions=[model_sync]),
    ])
