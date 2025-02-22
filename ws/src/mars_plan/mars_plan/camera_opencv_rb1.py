#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
import time

class GoldDetector(Node):
    def __init__(self):
        super().__init__('gold_detector_rb1')

        # ✅ 멀티스레딩을 위한 콜백 그룹 추가
        self.callback_group = ReentrantCallbackGroup()

        # 📌 카메라 이미지 구독 (객체 감지)
        self.subscription = self.create_subscription(
            Image,
            '/tb1/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )

        # 📌 네비게이션 중단 서비스 클라이언트
        self.nav2_cancel_client = self.create_client(
            Empty,
            '/navigation2/cancel',
            callback_group=self.callback_group
        )

        # 📌 Odometry (위치 정보) 구독
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.callback_group)

        # 📌 위치 정보 발행 (로봇 2에게 전달)
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/robot1/position',
            10
        )

        # 📌 속도 명령 발행
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/tb1/cmd_vel',
            10
        )

        # ✅ Nav2 Waypoint 이동을 위한 액션 클라이언트 추가
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=self.callback_group
        )

        self.bridge = CvBridge()
        self.frame_width = 640
        self.frame_height = 480
        self.nav2_active = True
        self.current_pose = PoseStamped()
        self.object_detected = False  # 객체 감지 여부 플래그

    def cancel_nav2(self):
        """📌 Nav2 탐색을 중단하는 함수"""
        if self.nav2_active:
            self.get_logger().info("🔴 Nav2 중단 요청!")
            req = Empty.Request()
            future = self.nav2_cancel_client.call_async(req)
            future.add_done_callback(self.nav2_stopped_callback)

    def nav2_stopped_callback(self, future):
        """📌 Nav2 중단 후 로봇 정지"""
        self.get_logger().info("🛑 Nav2 중단 완료, 로봇 정지!")
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_twist)
        time.sleep(0.2)  # 🔥 Nav2 종료 후 0.2초 대기
        self.nav2_active = False

    def send_waypoint_goal(self, x, y):
        """📌 특정 좌표(Waypoint)로 이동하는 함수"""
        self.get_logger().info(f"🚀 Waypoint 이동 시작: X={x}, Y={y}")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0  # 방향 (기본값)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.nav_to_pose_client.wait_for_server()
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """📌 Waypoint 도착 여부 확인"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("❌ 목표 지점 거부됨!")
            return

        self.get_logger().info("✅ 목표 지점 이동 시작!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """📌 Waypoint 이동 완료 콜백"""
        self.get_logger().info("🏁 Waypoint 도착 완료!")

    def odom_callback(self, msg):
        """📌 Odometry 데이터를 받아 현재 위치 저장"""
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def image_callback(self, msg):
        """📌 카메라에서 이미지를 받아 객체 탐지 후 동작 수행"""
        if self.object_detected:
            return  # 이미 감지한 경우 중복 탐지 방지

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_contour = max(contours, key=cv2.contourArea, default=None)

        if largest_contour is not None:
            self.object_detected = True
            if self.nav2_active:
                self.cancel_nav2()

            x, y, w, h = cv2.boundingRect(largest_contour)
            height_percentage = (h / self.frame_height) * 100

            if height_percentage >= 60:
                self.get_logger().info("🛑 멈춤! 로봇2에게 위치 전송")
                self.publish_position()

                # ✅ 객체 감지 후 특정 Waypoint로 이동
                time.sleep(3)
                self.send_waypoint_goal(1.5, -2.0)

        cv2.imshow("Gold Detection", mask)
        cv2.waitKey(1)

    def publish_position(self):
        """📌 로봇의 현재 위치를 다른 노드에 전송"""
        self.get_logger().info("📡 로봇1 위치 전송!")
        self.pose_publisher.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    node = GoldDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# import numpy as np
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry
# from std_srvs.srv import Empty
# import time

# class GoldDetector(Node):
#     def __init__(self):
#         super().__init__('gold_detector_rb1')

#         self.subscription = self.create_subscription(
#             Image,
#             '/tb1/camera/image_raw',
#             self.image_callback, 10
#         )
        
#         self.nav2_cancel_client = self.create_client(
#             Empty,
#             '/navigation2/cancel'
#         )
#         self.odom_subscriber = self.create_subscription(
#             Odometry, '/odom', self.odom_callback, 10)
        
#         self.pose_publisher = self.create_publisher(
#             PoseStamped,
#             '/robot1/position',
#             10)
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist,'/tb1/cmd_vel', 10)
        
#         self.bridge = CvBridge()
#         self.frame_width = 640
#         self.frame_height = 480
#         self.nav2_active = True
#         self.current_pose = PoseStamped()

#     def cancel_nav2(self):
#         if self.nav2_active:
#             self.get_logger().info("🔴 Nav2 중단 요청!")
#             req = Empty.Request()
#             future = self.nav2_cancel_client.call_async(req)
            
#             # 🔥 Nav2 중단 후 비동기적으로 처리
#             future.add_done_callback(self.nav2_stopped_callback)

#     def nav2_stopped_callback(self, future):
#         self.get_logger().info("🛑 Nav2 중단 완료, 로봇 정지!")
#         stop_twist = Twist()
#         stop_twist.linear.x = 0.0
#         stop_twist.angular.z = 0.0
#         self.cmd_vel_publisher.publish(stop_twist)
#         time.sleep(0.2)  # 🔥 Nav2 종료 후 0.2초 대기
#         self.nav2_active = False

#     def odom_callback(self, msg):
#         self.current_pose.header = msg.header
#         self.current_pose.pose = msg.pose.pose
        
#     def publish_position(self):
#         self.get_logger().info("📡 로봇1 위치 전송!")
#         self.pose_publisher.publish(self.current_pose)

#     def image_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         lower_yellow = np.array([20, 100, 100])
#         upper_yellow = np.array([35, 255, 255])
#         mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         largest_contour = max(contours, key=cv2.contourArea, default=None)
        
#         #if largest_contour is not None and cv2.contourArea(largest_contour) > 500: # <- 조건이 이상할수 있음, 
#         if largest_contour is not None: # <- 조건이 이상할수 있음, 
#             if self.nav2_active:  # ✅ Nav2가 실행 중일 때만 중단 요청
#                 self.cancel_nav2()
            
#             x, y, w, h = cv2.boundingRect(largest_contour)
#             center_x = x + w // 2
#             screen_center_x = self.frame_width // 2
#             offset_x = center_x - screen_center_x
#             twist_msg = Twist()

#             if not self.nav2_active:
#                 # 📌 Nav2 종료 후 즉시 회전 동작
#                 if abs(offset_x) > 20:
#                     twist_msg.angular.z = 0.2 if offset_x < 0 else -0.2
#                 else:
#                     twist_msg.angular.z = 0.0

#                 height_percentage = (h / self.frame_height) * 100

#                 # 📌 바운딩 박스 크기가 60% 이상이면 정지 & 로봇2에게 위치 전송
#                 if height_percentage >= 60:
#                     twist_msg.linear.x = 0.0
#                     self.get_logger().info("🛑 멈춤! 로봇2에게 위치 전송")
#                     self.publish_position()
#                 else:
#                     twist_msg.linear.x = 0.1  # 계속 접근

#                 self.cmd_vel_publisher.publish(twist_msg)

#             # 바운딩 박스 그리기
#             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
#             cv2.putText(frame, "Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#             detection_percentage = (h / self.frame_height) * 100
#             cv2.putText(frame, f"Detected {detection_percentage:.1f}%", (x, y - 10), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
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


# import rclpy
# from rclpy.node import Node
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# import numpy as np
# from geometry_msgs.msg import Twist, PoseStamped
# #  PoseStamped
# #   로봇의 현재 위치와 자세(Pose)를 포함하는 메시지 타입으로, 위치 데이터를 ROS2에서 처리할 때 사용
# from nav_msgs.msg import Odometry
# from std_srvs.srv import Empty
# import time

# '''
# PoseStamped
# header:
#   stamp: <타임스탬프>
#   frame_id: "map" (또는 "odom", "base_link" 등 좌표계 기준)
# pose:
#   position:
#     x: <X 좌표>
#     y: <Y 좌표>
#     z: <Z 좌표>
#   orientation:
#     x: <쿼터니언 회전 값>
#     y: <쿼터니언 회전 값>
#     z: <쿼터니언 회전 값>
#     w: <쿼터니언 회전 값>

# ✔ position.x, position.y, position.z → 로봇의 현재 위치 좌표
# ✔ orientation.x, y, z, w → 로봇의 현재 회전 정보(쿼터니언 방식)
# ✔ header.frame_id → 좌표계를 의미 (map, odom, base_link)
# ✔ header.stamp → 현재 시간 (ROS 타임스탬프)
    
# '''

# ---------------------------------------------

# # new_ver_20일 2시
# '''
# 📌 1. 동작 방식
# OpenCV로 물체(바운딩 박스)가 감지되면 Nav2를 중단
# 바운딩 박스가 화면 중심에 정렬되도록 로봇 회전
# 바운딩 박스 크기가 60%가 될 때까지 전진
# 목표 크기에 도달하면 멈추고 위치 전송
# '''

# class GoldDetector(Node):
#     def __init__(self):
#         super().__init__('gold_detector_rb1')

#         self.subscription = self.create_subscription(
#             Image,
#             '/tb1/camera/image_raw',
#             self.image_callback, 10
#         )
        
#         self.nav2_cancel_client = self.create_client(
#             Empty,
#             '/navigation2/cancel'
#         )
#         # 서비스 준비될 때까지 대기
#         while not self.nav2_cancel_client.wait_for_service(timeout_sec=2.0):
#             self.get_logger().warn("⏳ Nav2 중단 서비스 대기 중...")

#         self.odom_subscriber = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback, 10)
        
#         self.pose_publisher = self.create_publisher(
#             PoseStamped,
#             '/robot1/position',
#             10)
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist,
#             '/tb1/cmd_vel',
#             10)
        
#         self.bridge = CvBridge()
#         self.frame_width = 640
#         self.frame_height = 480
#         self.nav2_active = True
#         self.current_pose = PoseStamped()

#     def cancel_nav2(self):
#         if self.nav2_active:
#             self.get_logger().info("🔴 Nav2 중단 요청 전송 중...")

#             future = self.nav2_cancel_client.call_async(req)

#             # 🔥 Nav2가 중단될 때까지 동기적으로 대기
#             rclpy.spin_until_future_complete(self, future)

#             if future.result() is not None:
#                 self.get_logger().info("✅ Nav2 중단 완료!")

#                 # 🚀 Nav2 종료 후 즉시 정지 명령 (로봇 멈추기)
#                 stop_twist = Twist()
#                 stop_twist.linear.x = 0.0
#                 stop_twist.angular.z = 0.0
#                 self.cmd_vel_publisher.publish(stop_twist)

#                 self.nav2_active = False  # ✅ Nav2가 종료되었음을 명확히 표시

#             else:
#                 self.get_logger().warn("⚠️ Nav2 중단 요청 실패! 다시 시도해야 할 수도 있음.")



#     def nav2_stopped_callback(self, future):
#         self.get_logger().info("🛑 Nav2 중단 완료, 로봇 정지!")
#         stop_twist = Twist()
#         stop_twist.linear.x = 0.0
#         stop_twist.angular.z = 0.0
#         self.cmd_vel_publisher.publish(stop_twist)
#         time.sleep(0.2)  # 🔥 Nav2 종료 후 0.2초 대기
#         self.nav2_active = False

#     def odom_callback(self, msg):
#         self.current_pose.header = msg.header
#         self.current_pose.pose = msg.pose.pose
        
#     def publish_position(self):
#         # 현재 위치를 다시 받아와서 전송
#         position_msg = PoseStamped()
#         position_msg.header.stamp = self.get_clock().now().to_msg()
#         position_msg.header.frame_id = "map"
#         position_msg.pose = self.current_pose.pose
        
#         self.get_logger().info(f"📡 로봇1 현재 위치 전송: x={position_msg.pose.position.x}, y={position_msg.pose.position.y}")
#         self.pose_publisher.publish(position_msg)

#     def image_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         lower_yellow = np.array([20, 100, 100])
#         upper_yellow = np.array([35, 255, 255])
#         mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         largest_contour = max(contours, key=cv2.contourArea, default=None)

#         if largest_contour is not None:
#             # 🔥 Nav2가 실행 중이라면 즉시 중단
#             if self.nav2_active:
#                 self.cancel_nav2()

#             # ✅ Nav2가 완전히 멈춘 후, 바운딩 박스 위치 조정
#             if not self.nav2_active:
#                 x, y, w, h = cv2.boundingRect(largest_contour)
#                 center_x = x + w // 2
#                 screen_center_x = self.frame_width // 2
#                 offset_x = center_x - screen_center_x
#                 twist_msg = Twist()

#                 # 📌 바운딩 박스가 중앙에 위치하도록 회전
#                 if abs(offset_x) > 20:  # 화면 중앙과의 오차가 20px 이상이면 회전
#                     twist_msg.angular.z = 0.2 if offset_x < 0 else -0.2
#                 else:
#                     twist_msg.angular.z = 0.0  # 중앙에 있으면 회전 중지

#                 height_percentage = (h / self.frame_height) * 100

#                 # 📌 바운딩 박스 크기가 60% 이상이면 정지 & 로봇2에게 위치 전송
#                 if height_percentage >= 60:
#                     twist_msg.linear.x = 0.0
#                     self.get_logger().info("🛑 멈춤! 로봇2에게 현재 위치 전송")
#                     self.publish_position()
#                 else:
#                     twist_msg.linear.x = 0.1  # 크기가 60% 미만이면 전진

#                 self.cmd_vel_publisher.publish(twist_msg)

#             # 바운딩 박스 그리기
#             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
#             cv2.putText(frame, "Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#             detection_percentage = (h / self.frame_height) * 100
#             cv2.putText(frame, f"Detected {detection_percentage:.1f}%", (x, y - 10), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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



