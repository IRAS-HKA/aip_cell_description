from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


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
        "use_fake_hardware",
        default_value="True",
        description="Start robot with fake hardware mirroring command to its states.",
    ))

    robot_ip = LaunchConfiguration("robot_ip")
    eki_io_port = LaunchConfiguration("eki_io_port")
    n_io = LaunchConfiguration("n_io")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    eki_io_node = Node(
        package="aip_bosch_gripper",
        executable="aip_bosch_gripper_node",
        parameters=[
            {"robot_ip": robot_ip,
             "eki_io_port": eki_io_port,
             "n_io": n_io}
        ],
        output='screen',
        condition=UnlessCondition(use_fake_hardware)
    )
    only_visualizer = Node(
        package="aip_bosch_gripper",
        executable="GripperVisualizerNode.py",
        name="gripper_visualizer_node",
        output='screen',
    )
    
    return LaunchDescription(declared_arguments + [eki_io_node, only_visualizer])
    