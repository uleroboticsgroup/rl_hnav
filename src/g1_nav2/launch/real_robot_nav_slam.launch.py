"""
real_nav2_slam_bringup.launch.py

Real-robot bringup (NO Gazebo / NO MuJoCo) that starts:
  0) (optional) rl_sar real robot node (locomotion controller / cmd_vel consumer)
  1) humanoid_nav_bridge real_robot_bridge.launch.py
       - /dog_odom -> /odom + TF(odom->base_frame)
       - static TF(base_frame->lidar_frame)
       - PointCloud2 -> /scan
  2) slam_toolbox (online_async)
  3) Nav2 (navigation_launch)
  4) RViz (optional)
  5) Save map on shutdown

Validated defaults from your probes:
  - /dog_odom.header.frame_id  = "odom"
  - /dog_odom.child_frame_id   = "robot_center"
  - /utlidar/cloud_livox_mid360.frame_id = "livox_frame"

So we default:
  odom_frame = "odom"
  base_frame = "robot_center"
  lidar_frame = "livox_frame"

Important:
  - use_sim_time MUST be false on real robot (unless you have /clock).
  - slam_toolbox should own map->odom (we keep publish_map_odom_identity=false in the bridge).
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # -------------------------
    # Core args
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    start_rviz = LaunchConfiguration("start_rviz")

    # -------------------------
    # Optional rl_sar auto-start
    # -------------------------
    start_rl_sar = LaunchConfiguration("start_rl_sar")
    network_interface = LaunchConfiguration("network_interface")
    cmd_vel_timeout_sec = LaunchConfiguration("cmd_vel_timeout_sec")

    # NOTE: This runs rl_sar on the PC side (where you launch everything).
    # It assumes your ethernet interface to the robot is correct.
    rl_sar_process = ExecuteProcess(
        cmd=[
            "ros2", "run", "rl_sar", "rl_real_g1_edu23",
            network_interface,
            "--ros-args",
            "-p", "hw_mode:=0",
            "-p", "navigation_mode:=true",
            "-p", ["cmd_vel_timeout_sec:=", cmd_vel_timeout_sec],
        ],
        output="screen",
        condition=IfCondition(start_rl_sar),
    )

    # -------------------------
    # Bridge args (pass-through)
    # -------------------------
    input_odom_topic = LaunchConfiguration("input_odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")

    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")

    # static TF base_frame -> lidar_frame (initial guess; tune later)
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    # cloud->scan tuning
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")
    range_min = LaunchConfiguration("range_min")
    range_max = LaunchConfiguration("range_max")
    transform_tolerance = LaunchConfiguration("transform_tolerance")

    # -------------------------
    # Paths
    # -------------------------
    maps_dir = os.path.join(os.path.expanduser("~"), "g1_ws", "maps")
    map_prefix = os.path.join(maps_dir, "")

    g1_nav2_share = get_package_share_directory("g1_nav2")
    nav2_params_path = os.path.join(g1_nav2_share, "params", "nav2_g1_slam_real.yaml")
    slam_params_path = os.path.join(g1_nav2_share, "params", "real_slam_toolbox_online_async.yaml")

    # -------------------------
    # Include: real_robot_bridge.launch.py
    # -------------------------
    # IMPORTANT: our final bridge launch no longer expects map_frame/base_link/etc.
    # It expects:
    #   use_sim_time, input_odom_topic, cloud_topic,
    #   odom_frame, base_frame, lidar_frame,
    #   x,y,z, roll,pitch,yaw,
    #   min_height,max_height, range_min,range_max, transform_tolerance
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("humanoid_nav_bridge"),
                "launch",
                "real_robot_bridge.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,

            "input_odom_topic": input_odom_topic,
            "cloud_topic": cloud_topic,

            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "lidar_frame": lidar_frame,

            "x": x, "y": y, "z": z,
            "roll": roll, "pitch": pitch, "yaw": yaw,

            "min_height": min_height,
            "max_height": max_height,
            "range_min": range_min,
            "range_max": range_max,
            "transform_tolerance": transform_tolerance,
        }.items(),
    )

    # -------------------------
    # SLAM toolbox (online async)
    # -------------------------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_path,

            # These are safe defaults; keep them unless you have evidence to change
            "scan_queue_size": "250",
            "transform_timeout": "0.5",
        }.items(),
    )

    # -------------------------
    # Nav2
    # -------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_path,
            "namespace": "",
            "use_namespace": "false",
        }.items(),
    )

    # -------------------------
    # RViz (optional)
    # -------------------------
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "rviz_launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_rviz),
    )

    # -------------------------
    # Logging
    # -------------------------
    log_params = LogInfo(
        msg=[
            "REAL ROBOT bringup | NAV2 params_file = ", nav2_params_path,
            " | SLAM params_file = ", slam_params_path,
        ]
    )

    # -------------------------
    # Save map on shutdown
    # -------------------------
    save_map_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        f"mkdir -p {maps_dir} && "
                        f"ros2 run nav2_map_server map_saver_cli -f {map_prefix}{map_name}",
                    ],
                    output="screen",
                )
            ]
        )
    )

    # -------------------------
    # Delays (chain)
    # -------------------------
    # Strategy:
    #  - start bridge early (it provides /odom, /tf, /scan)
    #  - then start SLAM (needs /scan and TF)
    #  - then Nav2 (needs map from SLAM and TF)
    #
    # If rl_sar is enabled, it runs in parallel, but does not block mapping.
    bridge_delayed = TimerAction(period=1.0, actions=[bridge_launch])
    slam_delayed = TimerAction(period=4.0, actions=[slam])
    nav2_delayed = TimerAction(period=15.0, actions=[nav2])

    return LaunchDescription([
        # -------------------------
        # Core args
        # -------------------------
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Real robot: keep false unless /clock exists",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="g1_map",
            description="Map base name saved on shutdown under ~/g1_ws/maps/",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz (true/false).",
        ),

        # -------------------------
        # Optional rl_sar auto-start
        # -------------------------
        DeclareLaunchArgument(
            "start_rl_sar",
            default_value="false",
            description="If true, auto-start rl_real_g1_edu23 on this PC",
        ),
        DeclareLaunchArgument(
            "network_interface",
            default_value="enx000000000000",
            description="Ethernet interface used by rl_real_g1_edu23 (PC side)",
        ),
        DeclareLaunchArgument(
            "cmd_vel_timeout_sec",
            default_value="0.6",
            description="cmd_vel timeout for rl_sar",
        ),

        # -------------------------
        # Bridge args (validated defaults)
        # -------------------------
        DeclareLaunchArgument("input_odom_topic", default_value="/dog_odom"),
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_livox_mid360"),

        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument(
            "base_frame",
            default_value="robot_center",
            description="Base frame (from /dog_odom child_frame_id)",
        ),
        DeclareLaunchArgument(
            "lidar_frame",
            default_value="livox_frame",
            description="LiDAR frame (from PointCloud2 header.frame_id)",
        ),

        # Static TF base_frame -> lidar_frame (tune later)
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        # pointcloud_to_laserscan tuning
        DeclareLaunchArgument("min_height", default_value="-0.2"),
        DeclareLaunchArgument("max_height", default_value="0.2"),
        DeclareLaunchArgument("range_min", default_value="0.2"),
        DeclareLaunchArgument("range_max", default_value="30.0"),
        DeclareLaunchArgument("transform_tolerance", default_value="0.1"),

        # -------------------------
        # Actions
        # -------------------------
        log_params,

        # Optional rl_sar start
        #rl_sar_process,

        # Bringup chain
        bridge_delayed,
        slam_delayed,
        nav2_delayed,

        # Optional visualization
        rviz,

        # Always save on shutdown
        #save_map_on_shutdown,
    ])
