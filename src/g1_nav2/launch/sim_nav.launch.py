from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    g1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("g1_gazebo"), "launch", "spawn_g1.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("g1_nav2"), "launch", "bringup.launch.py"])
        )
    )

    return LaunchDescription([g1_spawn, nav2])
