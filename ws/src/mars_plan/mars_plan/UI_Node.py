from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from mars_interface.msg import CotrolCMD
from rclpy.node import Node

class PubUserCMANDNode(Node):
    def __init__(self):
        super().__init__('PubUserCMANDNode')
        self.Command_publisher = self.create_publisher(CotrolCMD, 'CotrolCMD', 10)

    def publish_Command(self, robot, command):
        msg = CotrolCMD()
        msg.robot = robot
        msg.command = command
        self.Command_publisher.publish(msg)

from PyQt5.QtCore import pyqtSignal

class GoldDetector(Node):
    image_signal = pyqtSignal(np.ndarray)  # PyQt에서 UI 업데이트를 위한 신호

    def __init__(self, NAMESPACE="tb1"):
        super().__init__('gold_detector_IMG_' + NAMESPACE)
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image, '/' + NAMESPACE + '/camera/image_raw',
            self.image_callback,
            10)
        self.frame_width = 640
        self.frame_height = 480
        
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea, default=None)

        if largest_contour is not None:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            detection_percentage = (h / self.frame_height) * 100
            cv2.putText(frame, f"Detected {detection_percentage:.1f}%", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        self.image_signal.emit(frame)  # UI 업데이트를 위한 신호 전달


class ROS2Thread(QThread):
    image_signal_tb1 = pyqtSignal(np.ndarray)
    image_signal_tb2 = pyqtSignal(np.ndarray)

    def run(self):
        rclpy.init()
        self.node = PubUserCMANDNode()
        self.detector_tb1 = GoldDetector("tb1")
        self.detector_tb2 = GoldDetector("tb2")
        
        self.detector_tb1.image_signal = self.image_signal_tb1
        self.detector_tb2.image_signal = self.image_signal_tb2
        
        # MultiThreadedExecutor 사용
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.detector_tb1)
        executor.add_node(self.detector_tb2)

        executor.spin()  # 두 개의 노드 실행
        rclpy.shutdown()

