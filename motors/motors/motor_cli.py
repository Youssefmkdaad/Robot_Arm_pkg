#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from robot_interfaces.srv import MoveMotor
from robot_interfaces.msg import Motors

class MotorClientNode(Node):
    def __init__(self):
        super().__init__("motor_cli")

        self.declare_parameter("SrvName", "move_motor_1")
        self.SrvName = self.get_parameter("SrvName").value

        self.sub = self.create_subscription(Motors, "joint_steps", self.sub_callback, 10)

    def sub_callback(self, msg):
        for i, (steps, direction) in enumerate(zip(msg.steps, msg.dir), start=1):
            self.call_move_motor_server(f"move_motor_{i}", steps, direction)

    def call_move_motor_server(self, name, steps, direction):
        client = self.create_client(MoveMotor, name)

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for server: {name}")
            return

        request = MoveMotor.Request()
        request.steps = steps
        request.dir = direction

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_move_motor_response, name=name))

    def callback_move_motor_response(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(f"The new location of {name} is: {response.new_location}")
        except Exception as e:
            self.get_logger().error(f"Service call failed for {name}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down client node.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
