#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState


class FakeDriver(Node):
    def __init__(self):
        super().__init__('fake_driver')

        # joint states publisher
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)

        # init variable
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_states.position = [0.0, 0.0]

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)
    
    def publish_callback(self):
        curr_time = self.get_clock().now()

        # joint states
        self.joint_states.header.stamp = curr_time.to_msg()

        # publish
        self.pub_joint_states.publish(self.joint_states)

        # simulate wheel rotate
        self.joint_states.position[0] += 0.05
        self.joint_states.position[1] += 0.05


def main(args=None):
    rclpy.init(args=args)

    driver = FakeDriver()
    executor = MultiThreadedExecutor()
    rclpy.spin(driver, executor=executor)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
