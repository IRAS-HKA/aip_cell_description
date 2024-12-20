from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        "robot_ip",
        default_value="10.166.32.145",
        description="IP address by which the robot can be reached.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="True",
        description="Start robot with fake hardware mirroring command to its states.",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "eki_robot_port",
        default_value="54600",
        description="Port by which the robot can be reached."
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
        "prefix",
        default_value="",
        description="Prefix for every link and joint in the robot description."
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_package",
        default_value="aip_cell_description",
        description="Robot description package",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_file",
        default_value="aip_kr10_cell.xacro",
        description="Robot description file located in <robot_description_package>/urdf/ .",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "semantic_description_file",
        default_value="aip_kr10_cell.srdf",
        description="Semantic robot description file located in <robot_description_package>/config/ .",
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "robot_controller",
        default_value="gripper_controller",
        choices=["gripper_controller", "position_trajectory_controller", "joint_state_controller"],
        description="Robot controller to start.",
    ))

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    eki_robot_port = LaunchConfiguration("eki_robot_port")
    eki_io_port = LaunchConfiguration("eki_io_port")
    n_io = LaunchConfiguration("n_io")
    prefix = LaunchConfiguration("prefix")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    semantic_description_file = LaunchConfiguration("semantic_description_file")
    robot_controller = LaunchConfiguration("robot_controller")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare(robot_description_package), "urdf", robot_description_file]), " ",
            "use_fake_hardware:=", use_fake_hardware, " ",
            "robot_ip:=", robot_ip, " ",
            "eki_robot_port:=", eki_robot_port, " ",
            "prefix:=", prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), "config", semantic_description_file]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kuka_common_moveit_config'), 'launch', 'demo.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": robot_description_package,
            "robot_description_file": robot_description_file
        }.items(),
    )
    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    # gripper_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([FindPackageShare('zimmer_gp406n'), 'launch', 'zimmer_gp406_node.launch.py'])),
    #     launch_arguments={
    #         "robot_ip": robot_ip,
    #         "eki_io_port": eki_io_port,
    #         "n_io": n_io,
    #     }.items(),
    # )

    moveit_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moveit_wrapper'), 'launch', 'moveit_wrapper.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": robot_description_package,
            "robot_description_file": robot_description_file
        }.items(),
    )

    kinematics_yaml = load_yaml("kuka_common_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    servo_yaml = load_yaml("aip_cell_description", "config/kuka_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [moveit_launch, moveit_wrapper_launch, servo_node])
