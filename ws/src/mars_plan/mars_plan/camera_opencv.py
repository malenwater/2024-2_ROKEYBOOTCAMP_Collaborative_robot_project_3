
# new_ver_2

# nav2, 회전, 이동 추가

'''
시나리오는 Nav2가 실행되다가 바운딩 박스를 감지하면 Nav2를 중단하고, 물체 중심으로 맞추고, 일정 거리까지 접근 후 정지하는 구조

🚀 실행 흐름 (전체 시나리오)
1️⃣ ROS2 환경 실행 & 카메라 활성화

Gazebo 또는 실제 환경에서 Nav2가 실행되고 있음
로봇 카메라에서 실시간으로 영상을 받아옴
2️⃣ Nav2가 목표 지점으로 이동 (cmd_vel 퍼블리시)

Nav2가 /cmd_vel을 퍼블리시하여 로봇이 목표로 이동 중
Odom을 통해 현재 속도를 확인하며 Nav2의 이동 상태를 추적
3️⃣ 카메라에서 프레임 수신 (image_callback)

카메라가 프레임을 받아서 이미지 처리 시작
회색(광물) 영역을 필터링하여 바운딩 박스를 검출
가장 큰 바운딩 박스 찾기 (cv2.findContours())
4️⃣ 바운딩 박스가 감지됨 → Nav2 중단 (cancel_nav2())

바운딩 박스가 감지되면 Nav2 중단 요청 (/navigation2/cancel 서비스 호출)
이후 로봇이 직접 바운딩 박스 중심을 맞추고 접근하도록 변경
5️⃣ 바운딩 박스 중심이 화면 중앙으로 오도록 회전

바운딩 박스의 중심 좌표(center_x) 계산
화면의 중앙(screen_center_x)과 비교하여 오른쪽/왼쪽 회전
offset_x 값이 크면 회전 (angular.z 조절)
6️⃣ 바운딩 박스의 높이를 분석하여 이동 or 정지

바운딩 박스의 세로 크기(h)가 화면 높이의 60% 이상이면 정지
그렇지 않으면 일정 속도(linear.x = 0.1)로 접근
7️⃣ 로봇이 물체 가까이 접근하면 멈춤

바운딩 박스 높이가 60% 이상이면 linear.x = 0.0 (정지)
화면에 "STOP!" 표시
'''
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty  # Nav2 중단을 위한 서비스 요청

class GoldDetector(Node):
    def __init__(self):
        super().__init__('gold_detector')

        # 카메라 이미지 구독
        self.subscription = self.create_subscription(
            Image, 
            '/tb1/camera/image_raw',  # 로봇1의 카메라 사용
            self.image_callback, 
            10)

        # Nav2 중단을 위한 서비스 클라이언트 생성
        self.nav2_cancel_client = self.create_client(Empty, '/navigation2/cancel')

        # Nav2의 이동 상태 확인 (Odom 메시지 구독)
        self.odom_subscriber = self.create_subscription(
            Odometry, 
            '/odom',  # Nav2의 이동 상태
            self.odom_callback, 
            10)

        self.bridge = CvBridge()

        # 로봇 회전 및 정지 명령 퍼블리셔
        self.cmd_vel_publisher = self.create_publisher(Twist, '/tb1/cmd_vel', 10)

        # 카메라 화면 크기 저장
        self.frame_width = 640
        self.frame_height = 480
        self.nav2_active = True  # 초기 상태에서 Nav2 실행 중이라고 가정

    def cancel_nav2(self):
        """ Nav2를 중단하는 서비스 요청 """
        if self.nav2_active:
            self.get_logger().info("🔴 Nav2 중단 요청!")
            req = Empty.Request()
            self.nav2_cancel_client.call_async(req) # 서비스 클라이언트에게 req비동기 호출,
            self.nav2_active = False  # Nav2 중단 상태로 변경

    def odom_callback(self, msg):
        """ Nav2가 이동 중인지 확인 """
        speed = msg.twist.twist.linear.x
        if abs(speed) > 0.01:  
            self.nav2_active = True  # Nav2가 이동 중
        else:
            self.nav2_active = False  # Nav2가 정지 중

    def image_callback(self, msg):
        """ 카메라 영상에서 바운딩 박스를 찾고, Nav2를 중단 후 제어 """
        # ROS Image 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 첫 프레임에서 카메라 크기 업데이트
        if self.frame_width is None or self.frame_height is None:
            self.frame_height, self.frame_width = frame.shape[:2]

        # BGR -> HSV 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 회색 광물 범위 설정 (HSV 기준) ** 실험용,,
        lower_grey = np.array([0, 0, 50])
        upper_grey = np.array([180, 50, 200])

        # 노란색 (Gold) 필터링 값으로 변경 **
        #lower_grey = np.array([20, 100, 100])  # 낮은 HSV 값 (Yellow 시작)
        #upper_grey = np.array([35, 255, 255])  # 높은 HSV 값 (Yellow 끝)


        # 색상 필터링 (마스크 생성)
        mask = cv2.inRange(hsv, lower_grey, upper_grey)

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

        # bb 생성 -> nav
        if largest_contour is not None:
            # Nav2를 중단
            self.cancel_nav2()

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


# ------------------------------------------------------------------------------------------------------


# # new_ver
# # twist명령 추가
# '''
# 1. 로봇 회전 _ 바운딩 박스 중심이 화면 중앙에서 벗어나면 회전
# 2. 정지 조건 _ 바운딩 박스 높이가 화면 높이의 60% 이상이면 정지

# 대상) 회색 광물이라고 가정 => 문제, 배경이 회색이여서, 바운딩박스가 정확하게 그려지지 않는 문제 있음,
#  광물 색을 노란색으로 해야 바운딩박스가 정확하게 그려질 것,

#  📌 Nav2 실행 중 Twist 메시지 (cmd_vel) 퍼블리시 가능 여부
# ✅ Nav2 실행 중에도 Twist 메시지를 퍼블리시할 수 있음
# ✅ 하지만 Nav2에서 속도를 자동으로 결정하기 때문에 직접 퍼블리시할 때 충돌 가능
# ✅ 해결 방법:

# Nav2에서 제공하는 cmd_vel을 중단하고 직접 퍼블리시
# Nav2를 사용하면서도 Twist 메시지를 일부 수정 가능 (/cmd_vel 주입)



# '''


# import rclpy
# from rclpy.node import Node
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# import numpy as np
# from geometry_msgs.msg import Twist

# class GoldDetector(Node):
#     def __init__(self):
#         super().__init__('gold_detector')

#         # 카메라 이미지 구독
#         self.subscription = self.create_subscription(
#             Image, 
#             '/tb1/camera/image_raw',  # 로봇1의 카메라 사용
#             self.image_callback, 
#             10)
        
#         self.bridge = CvBridge()

#         # 로봇 회전 및 정지 명령 퍼블리셔
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/tb1/cmd_vel', 10)

#         # 카메라 화면 크기 저장 (초기값, 첫 프레임에서 업데이트됨)
#         self.frame_width = 640
#         self.frame_height = 480

#     def image_callback(self, msg):
#         # ROS Image 메시지를 OpenCV 이미지로 변환
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # 첫 프레임에서 카메라 크기 업데이트
#         if self.frame_width is None or self.frame_height is None:
#             self.frame_height, self.frame_width = frame.shape[:2]

#         # BGR -> HSV 변환
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#         # 회색 범위 설정 (HSV 기준)
#         lower_grey = np.array([0, 0, 50])
#         upper_grey = np.array([180, 50, 200])

#         # 색상 필터링 (마스크 생성)
#         mask = cv2.inRange(hsv, lower_grey, upper_grey)

#         # 컨투어(윤곽선) 찾기
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         # 바운딩 박스가 가장 큰 객체 찾기
#         largest_contour = None
#         max_area = 0

#         for contour in contours:
#             area = cv2.contourArea(contour)
#             if area > 500 and area > max_area:
#                 largest_contour = contour
#                 max_area = area

#         if largest_contour is not None:
#             # Bounding Box 계산
#             x, y, w, h = cv2.boundingRect(largest_contour)
#             center_x = x + w // 2
#             center_y = y + h // 2

#             # 박스 중심점 표시
#             cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

#             # 바운딩 박스 그리기
#             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

#             # 화면의 중앙 좌표
#             screen_center_x = self.frame_width // 2

#             # 바운딩 박스 중심이 중앙에서 얼마나 벗어났는지 계산
#             offset_x = center_x - screen_center_x

#             # 속도 메시지 생성
#             twist_msg = Twist()

#             # 📌 **조건 1: 중심이 화면 중앙에서 벗어나면 로봇 회전**
#             if abs(offset_x) > 20:  # 오차 허용 범위 (20픽셀)
#                 if offset_x < 0:
#                     twist_msg.angular.z = 0.2  # 왼쪽으로 회전
#                 else:
#                     twist_msg.angular.z = -0.2  # 오른쪽으로 회전
#             else:
#                 twist_msg.angular.z = 0.0  # 중앙이면 회전 중지

#             # 📌 **조건 2: 박스 크기가 화면 높이의 60% 이상이면 정지**
#             if h / self.frame_height >= 0.6:
#                 twist_msg.angular.z = 0.0  # 회전 중지
#                 twist_msg.linear.x = 0.0  # 정지
#                 cv2.putText(frame, "STOP!", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
#             else:
#                 twist_msg.linear.x = 0.1  # 일정 속도로 이동 가능

#             # 로봇에게 명령 전송
#             self.cmd_vel_publisher.publish(twist_msg)

#         # 결과 화면 출력
#         cv2.imshow("Original", frame)
#         cv2.imshow("Gold Detection", mask)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = GoldDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# ------------------------------------------------------------------------------------------------------
# old_ver


# import rclpy
# from rclpy.node import Node
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# import numpy as np

# '''
# Original
# 	원본 카메라 화면 + 회색(구) 물체 감지 시 초록색 박스 표시
# Gold Detection
#     회색 필터링 결과 (회색만 하얀색으로 보이고 나머지는 검정)
# '''

# class GoldDetector(Node):
#     def __init__(self):
#         super().__init__('gold_detector')

#         # 카메라 이미지 구독
#         self.subscription = self.create_subscription(
#             Image, 
#             '/tb2/camera/image_raw',  # 원하는 로봇 카메라 토픽
#             self.image_callback, 
#             10)
        
#         self.bridge = CvBridge()

#     def image_callback(self, msg):
#         # ROS Image 메시지를 OpenCV 이미지로 변환
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # BGR -> HSV 변환
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#         # 회색 범위 설정 (HSV 기준)
#         lower_grey = np.array([0, 0, 50])    # 낮은 채도(S)와 중간 명도(V)
#         upper_grey = np.array([180, 50, 200]) # 높은 채도(S) 범위 내에서 필터링

#         # 색상 필터링 (마스크 생성)
#         mask = cv2.inRange(hsv, lower_grey, upper_grey)

#         # 컨투어(윤곽선) 찾기
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         # 박스 및 구 감지
#         for contour in contours:
#             if cv2.contourArea(contour) > 500:  # 너무 작은 노이즈 제거
#                 # Bounding Box 구하기
#                 x, y, w, h = cv2.boundingRect(contour)

#                 # 최소 외접 원 구하기 (구형 감지용)
#                 (cx, cy), radius = cv2.minEnclosingCircle(contour)
#                 center = (int(cx), int(cy))
#                 radius = int(radius)

#                 # 구 형태인지 체크 (원의 크기와 사각형의 크기가 비슷하면 구로 판단)
#                 aspect_ratio = w / h
#                 circularity = (4 * np.pi * cv2.contourArea(contour)) / (cv2.arcLength(contour, True) ** 2)

#                 # 구 형태 조건 (비율이 1에 가깝고, 원의 반지름과 박스 크기가 비슷할 때)
#                 if 0.8 <= aspect_ratio <= 1.2 and circularity > 0.7:
#                     cv2.circle(frame, center, radius, (255, 0, 0), 2)  # 파란색 원 (구 형태 감지)
#                     cv2.putText(frame, "Sphere", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#                 else:
#                     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 초록색 박스 (구가 아닌 경우)
#                     cv2.putText(frame, "Not Sphere", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#         # 결과 화면 출력
#         cv2.imshow("Original", frame)
#         cv2.imshow("Gold Detection", mask)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = GoldDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
