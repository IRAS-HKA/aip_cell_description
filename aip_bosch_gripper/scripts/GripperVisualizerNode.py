#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from aip_interfaces.msg import MoveCylinders
import numpy as np
class GripperVisualizerNode(Node):

    def __init__(self):
        super().__init__('gripper_visualizer_node')
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            MoveCylinders,
            '/cylinder_trigger',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.joint_name_map = ('joint_cylinder_front_left', 'joint_cylinder_front_right', 'joint_cylinder_back_left', 'joint_cylinder_back_right')

    def callback(self, data: MoveCylinders):
        self.get_logger().info("Received MoveCylinders data")
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [self.joint_name_map[cylinder_id - 1] for cylinder_id in data.cylinder_ids]  # Replace with your joint names
        self.get_logger().info(f"Got joint names: {joint_state.name}")
        self.get_logger().info(f"Got extensions: {data.extensions}")
        joint_state.position = [float(extension) for extension in data.extensions] 

        self.get_logger().info(f"Publishing joint states: {joint_state.position} \n for joints: {joint_state.name}")
        self.joint_state_publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = GripperVisualizerNode()
    node.get_logger().info("Starting Gripper Visualizer Node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()