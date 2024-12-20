from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.166.32.145",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
        "eki_robot_port",
        default_value="54600",
        description="Port by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="aip_cell_description",
            description="Robot description package",
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_file",
        default_value="kr10_cylinder.xacro",
        description="Robot description file located in <robot_description_package>/urdf/ .",
        )
    )

    declared_arguments.append(DeclareLaunchArgument(
        "semantic_description_file",
        default_value="kr10_cylinder.srdf",
        description="Semantic robot description file located in <robot_description_package>/config/ .",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    eki_robot_port = LaunchConfiguration("eki_robot_port")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    semantic_description_file = LaunchConfiguration("semantic_description_file")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kuka_common_moveit_config'), 'launch', 'demo.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": robot_description_package,
            "robot_description_file": robot_description_file,
            "semantic_description_file": semantic_description_file
        }.items(),
    )

    moveit_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moveit_wrapper'), 'launch', 'moveit_wrapper.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": robot_description_package,
            "robot_description_file": robot_description_file,
            "semantic_description_file" :semantic_description_file
        }.items(),
    )



    return LaunchDescription(declared_arguments + [moveit_launch, moveit_wrapper_launch])
