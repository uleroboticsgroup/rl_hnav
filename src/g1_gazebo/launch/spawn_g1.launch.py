import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    g1_gazebo_share = get_package_share_directory("g1_gazebo")
    g1_desc_share   = get_package_share_directory("g1_description")

    world = os.path.join(g1_gazebo_share, "worlds", "turtlebot3_house.world")
    urdf_path = os.path.join(g1_desc_share, "urdf", "g1_23dof.urdf")

    candidate_model_paths = [
        os.path.join(g1_gazebo_share, "models"),
        os.path.join(os.path.expanduser("~"), ".gazebo", "models"),
        os.path.join(g1_desc_share, "models"),
    ]
    model_paths = [p for p in candidate_model_paths if os.path.isdir(p)]

    # --- plugin path for your custom Gazebo model plugin (.so) ---
    custom_plugin_path = os.path.join(
        os.path.expanduser("~"),
        "g1_ws", "install", "gazebo_mujoco_pose_sync", "lib"
    )

    # Existing plugin path from env (might be empty)
    existing_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")

    # Compose final GAZEBO_PLUGIN_PATH
    # (Put our plugin first so Gazebo finds it even if paths conflict)
    final_plugin_path = custom_plugin_path
    if existing_plugin_path:
        final_plugin_path += ":" + existing_plugin_path

    # --- Gazebo ENV (actions launch) ---

    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    final_ld = custom_plugin_path + (":" + existing_ld if existing_ld else "")

    set_gazebo_env = [
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", final_plugin_path),
        SetEnvironmentVariable("LD_LIBRARY_PATH", final_ld),
    ]
    if model_paths:
        set_gazebo_env.append(
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", ":".join(model_paths))
        )
    # --- Start gzserver / gzclient ---

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver", "--verbose", world,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_state.so",
        ],
        output="screen",
    )
    gzclient = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="screen",
    )

    # --- Robot description ---
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description_content = f.read().strip()
    if not robot_description_content:
        raise RuntimeError(f"URDF is empty: {urdf_path}")

    robot_description_content = (
        robot_description_content
        .replace("package://g1_description", f"file://{g1_desc_share}")
        .replace("model://g1_description",  f"file://{g1_desc_share}")
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description_content
        }],
        output="screen",
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "g1", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0.92"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # ✅ Apply env vars BEFORE starting gzserver
        *set_gazebo_env,

        gzserver,
        gzclient,

        TimerAction(period=2.0, actions=[rsp]),
        TimerAction(period=4.0, actions=[spawn]),
    ])
