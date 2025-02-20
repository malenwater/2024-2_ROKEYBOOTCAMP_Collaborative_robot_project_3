import rclpy
from rclpy.node import Node
from mars_interface.msg import CotrolCMD
import queue
import threading
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class UserCommand(Node):
    def __init__(self, mainServer):
        super().__init__('UserCommand')
        self.get_logger().info('UserCommand start')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.mainServer = mainServer
        self.subscription = self.create_subscription(
            CotrolCMD,
            'CotrolCMD',
            self.return_callback,
            qos_profile
        )
        
        # 메시지 큐 생성
        self.msg_queue = queue.Queue()
        
        # 백그라운드 메시지 처리 스레드 실행

        self.runnig = False
        self.get_logger().info('UserCommand end')

    def return_callback(self, msg):
        """콜백에서 메시지를 큐에 저장"""
        self.get_logger().info(f'Queued message: {msg}')
        self.msg_queue.put(msg)  # 큐에 메시지 추가

    def process_messages(self):
        """큐에서 메시지를 하나씩 꺼내어 처리"""
        self.runnig = True
        self.get_logger().info(f'Processing message: start {self.runnig}')
        while self.runnig == True:
            # self.get_logger().info(f'Processing message: check')
            if not self.msg_queue.empty():
                msg = self.msg_queue.get()
                self.get_logger().info(f'Processing message: {msg}')
                robot = msg.robot
                command = msg.command
                self.mainServer.set_robot(robot)
                self.mainServer.set_command(command)
                
            time.sleep(0.5)  # 너무 빠르게 실행되는 것을 방지