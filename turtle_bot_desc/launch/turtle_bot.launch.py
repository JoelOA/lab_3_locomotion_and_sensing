from os.path import join

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration

# CONSTANTS
PACKAGE_NAME = "turtle_bot_desc"


def launch_setup(context: LaunchContext) -> list[Node]:
    """
    Function to setup the robot_state_publisher node

    Parameters
    ----------
    context : LaunchContext
        The launch context

    Returns
    -------
    list[Node]
        The robot_state_publisher node

    """

    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Get the launch configuration variables
    sim_mode = LaunchConfiguration("sim_mode").perform(context)
    
    # Process the xacro file to get the URDF
    xacro_file = join(pkg_share, "turtle_bot", "turtle_bot.urdf.xacro")
    robot_description = Command(
        [
            f"xacro {xacro_file} ",
            f"sim_mode:={sim_mode} ",
       ]
    )

    # Create a robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "sim_mode": sim_mode.lower() == "true",
            }
        ],
    )

    return [robot_state_publisher]


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to start the robot_state_publisher node

    Returns
    -------
    LaunchDescription
        The launch description

    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_mode",
                default_value="false",
                choices=["true", "false"],
                description="Determine if the robot is in simulation mode",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )