#------------------------------------------------------------------------------------------------------


# new_ver
# twist명령 추가
'''
1. 로봇 회전 _ 바운딩 박스 중심이 화면 중앙에서 벗어나면 회전
2. 정지 조건 _ 바운딩 박스 높이가 화면 높이의 60% 이상이면 정지

대상) 회색 광물이라고 가정 => 문제, 배경이 회색이여서, 바운딩박스가 정확하게 그려지지 않는 문제 있음,
 광물 색을 노란색으로 해야 바운딩박스가 정확하게 그려질 것,

 📌 Nav2 실행 중 Twist 메시지 (cmd_vel) 퍼블리시 가능 여부
✅ Nav2 실행 중에도 Twist 메시지를 퍼블리시할 수 있음
✅ 하지만 Nav2에서 속도를 자동으로 결정하기 때문에 직접 퍼블리시할 때 충돌 가능
✅ 해결 방법:

Nav2에서 제공하는 cmd_vel을 중단하고 직접 퍼블리시
Nav2를 사용하면서도 Twist 메시지를 일부 수정 가능 (/cmd_vel 주입)

# 이 파일은, gazebo 카메라가 정상적으로 이미지를 받아와서 바운딩 박스를 그리는지 확인하는데 사용할 예정!

'''


import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist

class GoldDetector(Node):
    def __init__(self):
        super().__init__('gold_detector')

        # 카메라 이미지 구독
        self.subscription = self.create_subscription(
            Image, 
            '/tb1/camera/image_raw',  # 로봇1의 카메라 사용
            self.image_callback, 
            10)
        
        self.bridge = CvBridge()

        # 로봇 회전 및 정지 명령 퍼블리셔
        self.cmd_vel_publisher = self.create_publisher(Twist, '/tb1/cmd_vel', 10)

        # 카메라 화면 크기 저장 (초기값, 첫 프레임에서 업데이트됨)
        self.frame_width = 640
        self.frame_height = 480

    def image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 첫 프레임에서 카메라 크기 업데이트
        if self.frame_width is None or self.frame_height is None:
            self.frame_height, self.frame_width = frame.shape[:2]

        # BGR -> HSV 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 회색 범위 설정 (HSV 기준)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        # 색상 필터링 (마스크 생성)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 컨투어(윤곽선) 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 바운딩 박스가 가장 큰 객체 찾기
        largest_contour = None
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500 and area > max_area:
                largest_contour = contour
                max_area = area

        if largest_contour is not None:
            # Bounding Box 계산
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2

            # 박스 중심점 표시
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # 바운딩 박스 그리기
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 화면의 중앙 좌표
            screen_center_x = self.frame_width // 2

            # 바운딩 박스 중심이 중앙에서 얼마나 벗어났는지 계산
            offset_x = center_x - screen_center_x

            # 속도 메시지 생성
            twist_msg = Twist()

            # 📌 **조건 1: 중심이 화면 중앙에서 벗어나면 로봇 회전**
            if abs(offset_x) > 20:  # 오차 허용 범위 (20픽셀)
                if offset_x < 0:
                    twist_msg.angular.z = 0.2  # 왼쪽으로 회전
                else:
                    twist_msg.angular.z = -0.2  # 오른쪽으로 회전
            else:
                twist_msg.angular.z = 0.0  # 중앙이면 회전 중지

            # 📌 **조건 2: 박스 크기가 화면 높이의 60% 이상이면 정지**
            if h / self.frame_height >= 0.6:
                twist_msg.angular.z = 0.0  # 회전 중지
                twist_msg.linear.x = 0.0  # 정지
                cv2.putText(frame, "STOP!", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                twist_msg.linear.x = 0.1  # 일정 속도로 이동 가능

            # 로봇에게 명령 전송
            self.cmd_vel_publisher.publish(twist_msg)

        # 결과 화면 출력
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

