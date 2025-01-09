#!/usr/bin/env python3
import rclpy
import lgpio
import time
from rclpy.node import Node
from robot_interfaces.srv import MoveMotor
from robot_interfaces.msg import MotorStatus

class MotorServerNode(Node):
    def __init__(self):
        super().__init__("motor_srv")
        
        self.declare_and_get_parameters()
        
        self.server_ = self.create_service(MoveMotor, self.ServerName, self.callback_move_motor)
        self.pub = self.create_publisher(MotorStatus, self.TopicName, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.my_location = 0.0
        self.ratio = 1.0

        self.setup_gpio()
        self.get_logger().info("MotorServerNode has started.")

    def declare_and_get_parameters(self):
        self.declare_parameter("SrvName", "move_motor_1")
        self.declare_parameter("MsgName", "motor_1_position")
        self.declare_parameter("StepPin", 17)
        self.declare_parameter("DirPin", 27)

        self.ServerName = self.get_parameter("SrvName").value
        self.TopicName = self.get_parameter("MsgName").value
        self.step_pin = self.get_parameter("StepPin").value
        self.dir_pin = self.get_parameter("DirPin").value

    def setup_gpio(self):
        self.gpio_chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.gpio_chip, self.dir_pin)
        lgpio.gpio_claim_output(self.gpio_chip, self.step_pin)

    def timer_callback(self):
        msg = MotorStatus()
        msg.my_location = self.my_location
        self.pub.publish(msg)

    def callback_move_motor(self, request, response):
        steps = request.steps
        direction = request.dir

        self.get_logger().info(f"Received move request: steps={steps}, direction={direction}")

        lgpio.gpio_write(self.gpio_chip, self.dir_pin, direction)

        self.perform_steps(steps)
        self.update_position(steps, direction)

        self.get_logger().info(f"Motor moved to new position: {self.my_location}")
        response.new_location = self.my_location
        return response

    def perform_steps(self, steps):
        step_delay = 0.0005
        for _ in range(steps):
            lgpio.gpio_write(self.gpio_chip, self.step_pin, 1)
            time.sleep(step_delay)
            lgpio.gpio_write(self.gpio_chip, self.step_pin, 0)
            time.sleep(step_delay)

    def update_position(self, steps, direction):
        step_angle = 1.8
        delta = self.ratio * step_angle * steps
        self.my_location += delta if direction else -delta

    def destroy_node(self):
        lgpio.gpio_write(self.gpio_chip, self.step_pin, 0)
        lgpio.gpio_write(self.gpio_chip, self.dir_pin, 0)
        lgpio.gpiochip_close(self.gpio_chip)
        self.get_logger().info("GPIO resources released.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
