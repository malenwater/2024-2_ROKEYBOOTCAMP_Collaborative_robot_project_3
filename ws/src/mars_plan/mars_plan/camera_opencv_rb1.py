# 이강태 250220

# camera_opencv.py -> rb1 로봇 입장 신규 생성
# 로봇2에게 로봇1의 현재 위치를 보내는 부분도 추가함

# 바운딩 박스가 일정 크기 이상이면 정지 후 /robot1/position 토픽을 통해 자신의 위치를 퍼블리시.

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

class GoldDetector(Node):
    def __init__(self):
        super().__init__('gold_detector_rb1')

        self.subscription = self.create_subscription(
            Image, '/tb1/camera/image_raw', self.image_callback, 10)
        
        self.nav2_cancel_client = self.create_client(Empty, '/navigation2/cancel')
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot1/position', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/tb1/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.frame_width = 640
        self.frame_height = 480
        self.nav2_active = True
        self.current_pose = PoseStamped()

    def cancel_nav2(self):
        if self.nav2_active:
            self.get_logger().info("🔴 Nav2 중단 요청!")
            # nav2끊는 부분인데 안끊킴 ㅠㅠ
            req = Empty.Request()
            self.nav2_cancel_client.call_async(req)
            # ----
            self.nav2_active = False

    def odom_callback(self, msg):
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        
    def publish_position(self):
        self.get_logger().info("📡 로봇1 위치 전송!")
        self.pose_publisher.publish(self.current_pose)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_contour = max(contours, key=cv2.contourArea, default=None)
        
        if largest_contour is not None and cv2.contourArea(largest_contour) > 500:
            self.cancel_nav2()
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            screen_center_x = self.frame_width // 2
            offset_x = center_x - screen_center_x
            twist_msg = Twist()
            
            if abs(offset_x) > 20:
                twist_msg.angular.z = 0.2 if offset_x < 0 else -0.2
            else:
                twist_msg.angular.z = 0.0
            
            if h / self.frame_height >= 0.3:
                twist_msg.linear.x = 0.0
                self.get_logger().info("🛑 멈춤! 로봇2에게 위치 전송")
                self.publish_position()
            else:
                twist_msg.linear.x = 0.1
            
            self.cmd_vel_publisher.publish(twist_msg)
            
            # 바운딩 박스 그리기
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # % 추가 
            detection_percentage = (h / self.frame_height) * 100
            cv2.putText(frame, f"Detected {detection_percentage:.1f}%", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow("Original", frame)
        cv2.imshow("Gold Detection", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GoldDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
