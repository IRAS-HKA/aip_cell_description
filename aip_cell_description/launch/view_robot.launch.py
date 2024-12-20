from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="aip_cell_description",
            description="Description package of the rrbot. Usually the argument is not set, \
        it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "kr10_cylinder.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "moveit.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Get SRDF file to publish robot_description_semantic
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("aip_cell_description"),
        'config',
        'kr10_cylinder.srdf'
    )

    with open(robot_description_semantic_path, 'r') as f:
        robot_description_semantic = f.read()

    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                       output='screen',
                       parameters=[{
                            'robot_description_semantic': robot_description_semantic,
                            'publish_robot_description_semantic': True,
                            # More params
                       }],
                       )

    delay_rviz_after_joint_state_publisher_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_publisher_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[rviz_node],
                ),
            ],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_node,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_publisher_node,
            move_group_node
        ]
    )

