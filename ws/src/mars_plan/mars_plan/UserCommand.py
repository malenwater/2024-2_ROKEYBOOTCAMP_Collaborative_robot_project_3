import rclpy
from rclpy.node import Node
from mars_interface.msg import CotrolCMD
import queue
import threading
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

'''
ROS2 노드(UserCommand)로 동작하며, CotrolCMD 메시지를 수신하여 MainServer에 전달하는 역할을 함.
메시지 큐(queue.Queue)를 사용하여 메시지를 비동기적으로 저장하고, 별도의 스레드에서 메시지를 처리

1️⃣ ROS2에서 "CotrolCMD" 메시지를 수신 → return_callback() 실행.
2️⃣ 콜백 함수가 메시지를 msg_queue에 저장.
3️⃣ 별도의 스레드(process_messages())에서 메시지를 하나씩 처리.
4️⃣ MainServer에 robot과 command를 전달.
5️⃣ MainServer가 로봇을 제어하는 동작 수행.

📌 현재 구조 (UserCommand 사용) ✅ UserCommand.py가 메시지를 받아 큐에 저장하고,
별도의 스레드에서 정리하여 MainServer에 전달.
✅ MainServer는 메시지 처리에 신경 쓰지 않고 로봇을 제어하는 데 집중할 수 있음.
'''


class UserCommand(Node):
    def __init__(self, mainServer): # <- Main이, mainserver인자로 넘어옴,
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

    # controlCMD 부분이 msg로 들어옴,
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