import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import SetParameter


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    publish_map_odom_identity = LaunchConfiguration(
    "publish_map_odom_identity")

    # Chemin cible de sauvegarde : ~/g1_ws/maps/<map_name>.yaml/.pgm
    maps_dir = os.path.join(os.path.expanduser("~"), "g1_ws", "maps")
    map_prefix = os.path.join(maps_dir, "")

    # ✅ Chemins ABSOLUS vers les YAML (évite les surprises avec substitutions)
    g1_nav2_share = get_package_share_directory("g1_nav2")
    nav2_params_path = os.path.join(g1_nav2_share, "params", "nav2_g1_slam.yaml")
    slam_params_path = os.path.join(g1_nav2_share, "params", "slam_toolbox_online_async.yaml")

    # Spawn robot
    g1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("g1_gazebo"),
                "launch",
                "spawn_g1.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # SLAM toolbox online async
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
            "scan_queue_size": "250",
            "transform_timeout": "0.5",
        }.items(),
    )

    # -----------------------------
    # rl_sar param_node (OBLIGATOIRE)
    # -----------------------------
    param_node = Node(
        package="rl_sar",
        executable="param_node",
        name="param_node",
        output="screen",
        parameters=[{
            "robot_name": "g1",         # ou le nom attendu par rl_sar (important)
            "gazebo_model_name": "g1",  # DOIT matcher le -model g1 du spawn
        }],
    )

    # -----------------------------
    # rl_sar locomotion (MuJoCo)
    # -----------------------------
    rl_sar_node = Node(
        package="rl_sar",
        executable="rl_sim_mujoco",    # si l'exécutable existe côté install
        name="rl_sim_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        additional_env={
            "RL_SAR_NAV_MODE": "1",   # force navigation_mode = true
        },
    )

    rl_mujoco = ExecuteProcess(
        cmd=[
            "ros2", "run", "rl_sar", "rl_mujoco", "g1", "scene_29dof",
            "--ros-args",
            "-p", "navigation_mode:=true",
            "-p", "cmd_vel_timeout_sec:=0.6",
            "-p", "cmd_vel_topic:=/cmd_vel",
        ],
        output="screen",
    )

    # -----------------------------
    # MuJoCo → Nav2 bridge
    # -----------------------------
    '''
        mujoco_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("humanoid_nav_bridge"),
                    "launch",
                    "mujoco_to_nav2_bridge.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "mujoco_pose_topic": "/mujoco/base_pose",
                "odom_topic": "/mujoco/odom",
            }.items(),
        )
    '''
    mujoco_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("humanoid_nav_bridge"),
                "launch",
                "mujoco_to_gazebo_bridge.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "mujoco_pose_topic": "/mujoco/base_pose",
            "odom_topic": "/odom",   
            "publish_map_odom_identity": publish_map_odom_identity,
        }.items(),
    )

    # Nav2 bringup (navigation) - PAS de namespace pour l’instant (debug clair)
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_path,
            # ✅ force no namespace (puisque ton YAML n’est pas sous nav2:)
            "namespace": "",
            "use_namespace": "false",
        }.items()
    )
    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "rviz_launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ✅ Log clair des fichiers réellement passés
    log_params = LogInfo(msg=[
        "NAV2 params_file = ", nav2_params_path,
        " | SLAM params_file = ", slam_params_path
    ])

    # ✅ Dump auto des params du controller_server (preuve)
    # (On le lance après Nav2, donc on le delay un peu)
    dump_controller_params = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "echo '--- DUMP /controller_server params ---' && "
            "ros2 param dump /controller_server > /tmp/controller_server_params.yaml && "
            "echo 'saved: /tmp/controller_server_params.yaml' && "
            "grep -n 'FollowPath\\.critics\\|transform_tolerance\\|cmd_vel_topic\\|goal_checker' /tmp/controller_server_params.yaml | head -n 80 || true"
        ],
        output="screen",
    )

    # Sauvegarde auto de map au shutdown
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

    # Delays (simple et efficace)
    slam_delayed = TimerAction(period=10.0, actions=[slam])
    nav2_delayed = TimerAction(period=16.0, actions=[nav2])
    dump_delayed = TimerAction(period=20.0, actions=[dump_controller_params])
    '''
    rl_sar_delayed = TimerAction(
        period=3.0,
        actions=[param_node, rl_sar_node],
    )
    '''
    rl_sar_delayed = TimerAction(period=3.0, actions=[rl_mujoco])
    bridge_delayed = TimerAction(
        period=2.0,
        actions=[mujoco_bridge],
    )

    '''
        return LaunchDescription([
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_name", default_value="g1_map"),

            log_params,

            # 1) Gazebo + robot figé (odom / scan / tf OK)
            g1_spawn,

            # 2) rl_sar (param_node + locomotion MuJoCo)
            #rl_sar_delayed,

            # 3) SLAM
            slam_delayed,

            # 4) Nav2
            nav2_delayed,

            # 5) Debug params Nav2
            dump_delayed,

            # 6) RViz
            rviz,
        ])
    '''

    # -----------------------------
    # GROUP 1: Bridge en WALL time (Option A)
    # -----------------------------
    bridge_group = GroupAction([
        SetParameter(name="use_sim_time", value=False),
        bridge_delayed,   # ton IncludeLaunchDescription du bridge
    ])

    # -----------------------------
    # GROUP 2: Tout le reste suit /clock
    # -----------------------------
    sim_group = GroupAction([
        SetParameter(name="use_sim_time", value=True),

        # 1) Gazebo
        g1_spawn,

        # 4) SLAM
        slam_delayed,

        # 5) Nav2
        nav2_delayed,

        # 6) Debug
        dump_delayed,

        # 7) RViz
        rviz,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),  
        DeclareLaunchArgument("map_name", default_value="g1_map"),
        DeclareLaunchArgument(
            "publish_map_odom_identity",
            default_value="false",
            description="Publish static TF map->odom identity (debug SLAM)"
        ),

        log_params,

        # ✅ Bridge d'abord et hors sim_time true
        bridge_group,

        # ✅ Le reste en sim time
        sim_group,
    ])

