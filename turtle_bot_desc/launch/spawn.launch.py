from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "turtle_bot_desc"
WORLD_PACKAGE_NAME = "ashbot_world"

def launch_setup(context: LaunchContext) -> list:
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    world_package_share = FindPackageShare(package=WORLD_PACKAGE_NAME).find(WORLD_PACKAGE_NAME)

    world = LaunchConfiguration("world").perform(context)
    
    #Spawn robot at the given position
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    # robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(pkg_share, "launch", "turtle_bot.launch.py")]),
        launch_arguments={
            "sim_mode": "true",
        }.items(),
    )

    # Gazebo launch file
    world_filepath = join(world_package_share, "worlds", f"{world}.world")
    print(world_filepath)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
                "gz_args": ["-r -v4 ", world_filepath],
                "on_exit_shutdown": "true",
        }.items(),
    )

    # spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "turtle_bot",
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
            "-R", str(roll),
            "-P", str(pitch),
            "-Y", str(yaw),
        ],
        output="screen",
    )

    # ROS-Gazebo bridge
    bridge_params = join(pkg_share, "config", "gz_bridge.yaml")

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    launch_nodes = [rsp, gazebo, spawn_entity, ros_gz_bridge]

    return launch_nodes

def generate_launch_description() -> LaunchDescription:

    return LaunchDescription(
        [
            DeclareLaunchArgument("x", default_value="0", description="X position"),
            DeclareLaunchArgument("y", default_value="0", description="Y position"),
            DeclareLaunchArgument("z", default_value="0.056", description="Z position"),
            DeclareLaunchArgument("roll", default_value="0", description="Roll"),
            DeclareLaunchArgument("pitch", default_value="0", description="Pitch"),
            DeclareLaunchArgument("yaw", default_value="0", description="Yaw"),
            DeclareLaunchArgument("world", default_value="wall_arena", description="World name"),
            OpaqueFunction(function=launch_setup)
        ]
    )
