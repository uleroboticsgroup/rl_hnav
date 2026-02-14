from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    world = PathJoinSubstitution([
        FindPackageShare("g1_gazebo"),
        "worlds",
        "space.world"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "verbose": "true"
        }.items()
    )
    '''
    robot_description = ParameterValue(
    Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([FindPackageShare("g1_description"), "urdf", "g1_fake.urdf.xacro"])
    ]),
    value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
        output="screen"
    )


    urdf_path = os.path.join(
        FindPackageShare("g1_description").find("g1_description"),
        "urdf",
        "g1_23dof.urdf"
    )

    with open(urdf_path, "r") as f:
        robot_description_content = f.read()
    robot_description = {"robot_description": robot_description_content}

    '''
    
    # Chemin ABS du package share
    g1_share = FindPackageShare("g1_description").find("g1_description")
    urdf_path = os.path.join(g1_share, "urdf", "g1_23dof.urdf")

    # Lecture URDF
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description_content = f.read().strip()

    if not robot_description_content:
        raise RuntimeError(f"URDF is empty: {urdf_path}")

    # Remplacements pour RViz (package://) et Gazebo (model://)
    # NOTE: on mappe les deux vers file://<ABS_SHARE>
    robot_description_content = (
        robot_description_content
        .replace("package://g1_description", f"file://{g1_share}")
        .replace("model://g1_description",  f"file://{g1_share}")
    )

    robot_description = {"robot_description": robot_description_content}
    

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
        output="screen"
    )


    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "g1", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0.0"],
        output="screen"
    )
    
    rsp_delayed = TimerAction(period=5.0, actions=[rsp])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gazebo,
        rsp_delayed,
        spawn
    ])

