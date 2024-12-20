#!/usr/bin/env python3

import rclpy
import random
from aip_interfaces.msg import MoveCylinders
from aip_interfaces.srv import VizCloseGripper, VizOpenGripper

def get_random_cylinder_ids():
    return random.sample(range(1, 5), random.randint(1,4))

def get_random_extensions(num_cylinders):
    return [random.uniform(0, 0.15) for _ in range(num_cylinders)]

def publish_random_cylinders(node, publisher):
    msg = MoveCylinders()
    msg.cylinder_ids = get_random_cylinder_ids()
    node.get_logger().info("Got Random cylinder ids")
    msg.extensions = get_random_extensions(len(msg.cylinder_ids))
    node.get_logger().info("Got Random extensions")
    node.get_logger().info(f"Publishing MoveCylinders: {msg}")
    publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gripper_viz_client', parameter_overrides=[])
    node.get_logger().info("Starting Gripper Visualizer Client")
    publisher = node.create_publisher(MoveCylinders, '/cylinder_trigger', 1)
    node.get_logger().info("Publishing random cylinders every 5 seconds")
    timer = node.create_timer(5, lambda: publish_random_cylinders(node, publisher))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def test_service(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gripper_viz_client', parameter_overrides=[])
    node.get_logger().info("Starting Gripper Visualizer Client")
    client = node.create_client(VizCloseGripper, '/viz_close_gripper')
    request = VizCloseGripper.Request()
    request.cylinders.cylinder_ids = get_random_cylinder_ids()
    request.cylinders.extensions = [0.14,] * len(request.cylinders.cylinder_ids)
    node.get_logger().info(f"Sending CloseGripper request: {request}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"Response: {future.result()}")
    else:
        node.get_logger().error(f"Service call failed: {future.exception()}")
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    # main()
    test_service()