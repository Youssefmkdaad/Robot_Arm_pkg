import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MotorStatus, Motors
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.publisher_ = self.create_publisher(Motors, "joint_steps", 10)
        self.gazebo_pub = self.create_publisher(JointTrajectory, "set_joint_trajectory", 50)
        self.motor_positions = [0.0] * 4
        self.current_positions = [0.0] * 6
        self.ratio = 1.0
        self.flag = 0

        self.subscribers = [
            self.create_subscription(MotorStatus, f"motor_{i + 1}_position", self.create_callback(i), 10)
            for i in range(4)
        ]

        self.timer = self.create_timer(5.0, self.publish_motor_commands)

    def create_callback(self, index):
        def callback(msg):
            self.motor_positions[index] = msg.my_location
        return callback

    def publish_motor_commands(self):

        positions = [
            [0, -40, 0, -50, 0.0, 0.0],
            [0, 0, 0, 0, 0.015, 0.015],
            [-45, 40, 0, -30, 0, 0],
            [-45, -40, 0, 30, 0, 0],
            [0, 0, 0, 0, -0.015, -0.015],
            [90, 40, 0, 50, 0, 0],
        ]

        if self.flag == 0: self.flag = 1
        else: self.flag = 0

        for pos in positions:
            if self.flag == 0: pos[0] *= -1
            steps, dirs = self.calculate_inverse_kinematics(*pos)
            msg = Motors(steps=steps, dir=dirs)
            self.publisher_.publish(msg)

    def calculate_inverse_kinematics(self, x, y, z, orientation, f1, f2):

        angles = [np.deg2rad(val) for val in [x, -y, -z, orientation]]

        msg = JointTrajectory()
        msg.header.frame_id = "base_footprint"
        msg.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "finger_joint_1", "finger_joint_2"]

        step_increments = [angle / 50 for angle in angles] + [f1 / 50, f2 / 50]
        

        for _ in range(50):
            self.current_positions = [pos + inc for pos, inc in zip(self.current_positions, step_increments)]
            point = JointTrajectoryPoint(positions=self.current_positions)
            msg.points.append(point)
            self.gazebo_pub.publish(msg)
            msg.points.clear()
            time.sleep(1 / 50)

        self.get_logger().info(f"Calculated joint angles: {self.current_positions}")

        steps = [int(-angle * self.ratio / 1.8) for angle in self.current_positions[:4]]
        dirs = [step >= 0 for step in steps]  
        steps = [abs(step) for step in steps]

        return steps, dirs


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
