from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        "robot_ip",
        default_value="10.166.32.145",
        description="IP address by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "eki_io_port",
        default_value="54601",
        description="Port by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "n_io",
        default_value="2",
        description="Port by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "only_visualize",
        default_value="false",
        description="If true, the node will only visualize the gripper state."
    ))

    robot_ip = LaunchConfiguration("robot_ip")
    eki_io_port = LaunchConfiguration("eki_io_port")
    n_io = LaunchConfiguration("n_io")
    only_visualize = LaunchConfiguration("only_visualize")

    eki_io_node = Node(
        package="aip_bosch_gripper",
        executable="aip_bosch_gripper_node",
        parameters=[
            {"robot_ip": robot_ip,
             "eki_io_port": eki_io_port,
             "n_io": n_io}
        ],
        output='screen'
    )
    if not only_visualize:
        return LaunchDescription(declared_arguments + [
            eki_io_node, 
            Node(
                package="aip_bosch_gripper",
                executable="GripperVisualizerNode.py",
                name="gripper_visualizer_node",
                output='screen',
            )
        ])
    else:
        return LaunchDescription(declared_arguments + [
            Node(
                package="aip_bosch_gripper",
                executable="GripperVisualizerNode.py",
                name="gripper_visualizer_node",
                output='screen',
            )
        ])
