import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")

    # Chemin cible de sauvegarde : ~/g1_ws/maps/<map_name>.yaml/.pgm
    maps_dir = os.path.join(os.path.expanduser("~"), "g1_ws", "maps")
    map_prefix = os.path.join(maps_dir, "")  # juste pour être clair

    # Fichiers params
    pkg_g1_nav2 = FindPackageShare("g1_nav2")
    nav2_params = PathJoinSubstitution([pkg_g1_nav2, "params", "nav2_g1_slam.yaml"])
    slam_params = PathJoinSubstitution([pkg_g1_nav2, "params", "slam_toolbox_online_async.yaml"])

    # Spawn robot
    g1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("g1_gazebo"), "launch", "spawn_g1.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    # SLAM toolbox online async
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params,
        }.items()
    )

    # Nav2 (pendant SLAM)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items()
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "rviz_launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    # Sauvegarde automatique à la fermeture (Ctrl+C)
    save_map_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        "bash", "-lc",
                        # mkdir -p + map_saver_cli -f <prefix>
                        f"mkdir -p {maps_dir} && "
                        f"ros2 run nav2_map_server map_saver_cli -f {map_prefix}{map_name}"
                    ],
                    output="screen"
                )
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map_name", default_value="g1_map"),
        g1_spawn,
        slam,
        nav2,
        rviz,
        save_map_on_shutdown,
    ])
