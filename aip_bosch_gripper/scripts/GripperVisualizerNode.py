#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from aip_interfaces.msg import MoveCylinders
from aip_interfaces.srv import VizOpenGripper, VizCloseGripper

class GripperVisualizerNode(Node):
    def __init__(self):
        super().__init__('gripper_visualizer_node')
        self.gripper_controller_publisher = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.open_service = self.create_service(VizOpenGripper, '/viz_open_gripper', self.open_gripper_callback)
        self.close_service = self.create_service(VizCloseGripper, '/viz_close_gripper', self.close_gripper_callback)
        self.subscription = self.create_subscription(
            MoveCylinders,
            '/cylinder_trigger',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timeout = self.create_rate(0.1, self.get_clock())
        self.joint_name_map = ('joint_cylinder_front_left', 'joint_cylinder_front_right', 'joint_cylinder_back_left', 'joint_cylinder_back_right')

    def callback(self, data: MoveCylinders):
        self.get_logger().info("Received MoveCylinders data")
        cylinder_activations = Float64MultiArray()
        extensions = [0.0, 0.0, 0.0, 0.0]
        for cylinder_id in data.cylinder_ids:
            extensions[cylinder_id - 1] = data.extensions[data.cylinder_ids.index(cylinder_id)]
        cylinder_activations.data = extensions
        names = [self.joint_name_map[cylinder_id - 1] for cylinder_id in data.cylinder_ids]  # Replace with your joint names
        self.get_logger().info(f"Got joint names: {names}")
        self.get_logger().info(f"Got extensions: {data.extensions}")
        self.get_logger().info(f"Publishing goal states: {cylinder_activations.data} \n for joints: {names}")
        self.gripper_controller_publisher.publish(cylinder_activations)
    
    def open_gripper_callback(self, request, response):
        self.get_logger().info("Received OpenGripper request")
        cylinder_activations = Float64MultiArray()
        extensions = [0.0, 0.0, 0.0, 0.0]
        for cylinder_id in request.cylinders.cylinder_ids:
            extensions[cylinder_id - 1] = request.cylinders.extensions[request.cylinders.cylinder_ids.index(cylinder_id)]
        cylinder_activations.data = extensions
        names = [self.joint_name_map[cylinder_id - 1] for cylinder_id in request.cylinders.cylinder_ids]  # Replace with your joint names
        self.get_logger().info(f"Extending Cylinders:{names}")
        self.gripper_controller_publisher.publish(cylinder_activations)
        success = self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3))
        self.get_logger().info(f"Slept successfully: {success}")
        cylinder_activations.data = [0.0,] * 4
        self.get_logger().info(f"Retracting Cylinders:{names}")
        self.gripper_controller_publisher.publish(cylinder_activations)
        response.result = True
        return response

    def close_gripper_callback(self, request, response):
        self.get_logger().info("Received CloseGripper request")
        cylinder_activations = Float64MultiArray()
        extensions = [0.0, 0.0, 0.0, 0.0]
        for cylinder_id in request.cylinders.cylinder_ids:
            extensions[cylinder_id - 1] = request.cylinders.extensions[request.cylinders.cylinder_ids.index(cylinder_id)]
        cylinder_activations.data = extensions
        names = [self.joint_name_map[cylinder_id - 1] for cylinder_id in request.cylinders.cylinder_ids]  # Replace with your joint names
        self.get_logger().info(f"Extending Cylinders:{names}")
        self.gripper_controller_publisher.publish(cylinder_activations)
        success = self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3))
        self.get_logger().info(f"Slept successfully: {success}")
        cylinder_activations.data = [0.0,] * 4
        self.get_logger().info(f"Retracting Cylinders:{names}")
        self.gripper_controller_publisher.publish(cylinder_activations)
        response.result = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperVisualizerNode()
    node.get_logger().info("Starting Gripper Visualizer Node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()