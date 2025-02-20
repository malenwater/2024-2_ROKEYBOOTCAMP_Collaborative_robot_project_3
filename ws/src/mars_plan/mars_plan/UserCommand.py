import rclpy
from rclpy.node import Node
from mars_interface.msg import CotrolCMD


class UserCommand(Node):
    def __init__(self,mainServer):
        super().__init__('UserCommand')
        self.get_logger().info(f'UserCommand start')
        self.return_flag = False
        self.subscription = self.create_subscription(
            CotrolCMD,
            'CotrolCMD',
            self.return_callback,
            10
        )
        self.mainServer = mainServer
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'UserCommand end')
            
    def return_callback(self, msg):
        self.get_logger().info(f'Received return message in UserCommand: {msg}')
        robot = msg.robot
        command = msg.command
        self.mainServer.run_control(robot,command)