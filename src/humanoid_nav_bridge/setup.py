from setuptools import find_packages, setup

package_name = "humanoid_nav_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/humanoid_nav_bridge.launch.py",
            "launch/mujoco_to_gazebo_bridge.launch.py",
            "launch/real_robot_bridge.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jean Chrysostome Mayoko Biong",
    maintainer_email="icmayoko18@gmail.com",
    description=(
        "ROS 2 bridge for humanoid navigation: "
        "real robot odom -> TF + /odom, LiDAR cloud -> /scan, Nav2 compatible"
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # Simulation / MuJoCo
            "bridge_node = humanoid_nav_bridge.bridge_node:main",
            "mtog_bridge = humanoid_nav_bridge.mujoco_to_gazebo_node:main",
            "mujoco_to_gazebo_model = humanoid_nav_bridge.mujoco_to_gazebo_model:main",

            # Real robot
            "odom_tf_bridge = humanoid_nav_bridge.real_odom_tf_bridge:main",
            "real_state_to_odom_tf = humanoid_nav_bridge.real_state_to_odom_tf_node:main",

            # Scan timestamp fixer (IMPORTANT)
            "scan_restamper = humanoid_nav_bridge.scan_restamper:main",

            # Debug tools (optional)
            "cloud_frameid_probe = humanoid_nav_bridge.cloud_frameid_probe:main",
        ],
    },

)
