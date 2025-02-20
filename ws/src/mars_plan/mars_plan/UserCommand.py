import rclpy
from rclpy.node import Node
from mars_interface.msg import CotrolCMD
import queue
import threading
import time

class UserCommand(Node):
    def __init__(self, mainServer):
        super().__init__('UserCommand')
        self.get_logger().info('UserCommand start')
        
        self.mainServer = mainServer
        self.subscription = self.create_subscription(
            CotrolCMD,
            'CotrolCMD',
            self.return_callback,
            10
        )
        
        # 메시지 큐 생성
        self.msg_queue = queue.Queue()
        
        # 백그라운드 메시지 처리 스레드 실행
        self.worker_thread = threading.Thread(target=self.process_messages, daemon=True)
        self.worker_thread.start()

        self.get_logger().info('UserCommand end')

    def return_callback(self, msg):
        """콜백에서 메시지를 큐에 저장"""
        self.get_logger().info(f'Queued message: {msg}')
        self.msg_queue.put(msg)  # 큐에 메시지 추가

    def process_messages(self):
        """큐에서 메시지를 하나씩 꺼내어 처리"""
        while rclpy.ok():
            if not self.msg_queue.empty():
                msg = self.msg_queue.get()
                self.get_logger().info(f'Processing message: {msg}')
                robot = msg.robot
                command = msg.command
                self.mainServer.run_control(robot, command)
                time.sleep(0.1)  # 너무 빠르게 실행되는 것을 방지