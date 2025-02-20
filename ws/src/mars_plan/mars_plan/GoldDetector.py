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
from geometry_msgs.msg import PoseWithCovarianceStamped

class GoldDetector(Node):
    def __init__(self,MainServer , ORDER, NAMESPACE):
        super().__init__('gold_detector'+NAMESPACE)
        self.get_logger().info(f'gold_detector {NAMESPACE} start')
        self.MainServer = MainServer
        self.NAMESPACE = NAMESPACE
        self.ORDER = ORDER
        
        self.subscription = self.create_subscription(
            Image,  '/' + NAMESPACE + '/camera/image_raw', self.image_callback, 10)
        
        # self.nav2_cancel_client = self.create_client(Empty, '/navigation2/cancel')
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/' + NAMESPACE + '/amcl_pose', self.amcl_pose_callback, 10)
        
        
        # self.pose_publisher = self.create_publisher(PoseStamped, '/robot1/position', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/' + NAMESPACE + '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.frame_width = 640
        self.frame_height = 480
        self.nav2_active = True
        self.current_pose = PoseStamped()
        self.get_logger().info(f'gold_detector {NAMESPACE} end')
        self.x = None
        self.y = None
        
    # def cancel_nav2(self):
    #     if self.nav2_active:
    #         self.get_logger().info("🔴 Nav2 중단 요청!")
    #         # nav2끊는 부분인데 안끊킴 ㅠㅠ
    #         req = Empty.Request()
    #         self.nav2_cancel_client.call_async(req)
    #         # ----
    #         self.nav2_active = False
    
    def amcl_pose_callback(self, msg):
        # x와 y 값을 PoseWithCovarianceStamped 메시지에서 가져와 self.x, self.y에 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
    def odom_callback(self, msg):
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        
    # def publish_position(self):
    #     self.get_logger().info("📡 로봇1 위치 전송!")
    #     self.pose_publisher.publish(self.current_pose)

    def image_callback(self, msg):
        if self.MainServer.get_GoldDetector_FLAG(self.ORDER) == False:
            self.get_logger().info(f'get_GoldDetector_FLAG Faslse')
            return
        
        self.get_logger().info(f'get_GoldDetector_FLAG True')
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_contour = max(contours, key=cv2.contourArea, default=None)
        
        self.get_logger().info(f'get_GoldDetector_FLAG {largest_contour is not None}')
        if largest_contour is not None :
            # self.cancel_nav2()
            self.get_logger().info("check stop")
            self.MainServer.stop_NAV(self.ORDER)
            self.get_logger().info("check stop end")
            
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            screen_center_x = self.frame_width // 2
            offset_x = center_x - screen_center_x
            twist_msg = Twist()
            
            if abs(offset_x) > 20:
                twist_msg.angular.z = 0.08 if offset_x < 0 else -0.2
            else:
                twist_msg.angular.z = 0.0
            
            if h / self.frame_height >= 0.6:
                twist_msg.linear.x = 0.0
                self.get_logger().info(f"🛑 멈춤! {self.NAMESPACE}에게 위치 전송")
                self.MainServer.collect_robots(self.x,self.y,self.ORDER)
                self.MainServer.set_GoldDetector_FLAG(self.ORDER,False)
                # self.publish_position()
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
        
        # cv2.imshow("Original", frame)
        # cv2.imshow("Gold Detection", mask)
        # cv2.waitKey(1)
        self.get_logger().info(f'image_callback end')

def main(args=None):
    rclpy.init(args=args)
    node = GoldDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
