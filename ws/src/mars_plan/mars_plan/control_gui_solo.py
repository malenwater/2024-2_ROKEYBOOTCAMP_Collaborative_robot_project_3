'''
0.실행순서
cd ~/study_ws
source install/setup.bash
ros2 launch mars_plan mars.launch.py


1.실행순서
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true


2.실행순서
cd /home/kante/Documents/GitHub/2024-2_ROKEYBOOTCAMP_Collaborative_robot_project_3/ws/src/mars_plan/mars_plan
source /home/kante/Documents/GitHub/2024-2_ROKEYBOOTCAMP_Collaborative_robot_project_3/ws/install/setup.bash
python3 control_gui_solo.py


'''


import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class ControlGUISolo(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'control_gui_solo')
        QWidget.__init__(self)
        
        self.initUI()
        
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/navigate_to_pose',  # ✅ 변경: goal_pose → navigate_to_pose
            10)
        
        self.current_pose = None
        
    def initUI(self):
        layout = QVBoxLayout()
        
        self.label = QLabel("현재 위치: Unknown")
        layout.addWidget(self.label)
        
        self.btn_get_pose = QPushButton("현재 위치 출력")
        self.btn_get_pose.clicked.connect(self.show_current_pose)
        layout.addWidget(self.btn_get_pose)
        
        self.btn_move = QPushButton("10,10 위치로 이동")
        self.btn_move.clicked.connect(self.move_to_goal)
        layout.addWidget(self.btn_move)
        
        self.setLayout(layout)
        self.setWindowTitle("Single Robot Control GUI")
        self.show()
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def show_current_pose(self):
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            self.label.setText(f"현재 위치: x={x:.2f}, y={y:.2f}")
            print(f"✅ 현재 위치: x={x:.2f}, y={y:.2f}")  # 디버깅 출력
        else:
            self.label.setText("현재 위치를 가져오는 중...")
            print("⚠ 현재 위치를 가져오는 중...")  # 디버깅 출력
    
    def move_to_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = 10.0
        goal_msg.pose.position.y = 10.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_publisher.publish(goal_msg)
        self.label.setText("목표 위치 (10,10)로 이동 중...")
        print("🚀 목표 위치 (10,10)로 이동 명령 전송!")  # 디버깅 출력

def ros2_thread():
    rclpy.spin(gui)  # ROS2 노드 실행 (별도 스레드에서 실행)

if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    gui = ControlGUISolo()

    # ROS2 노드를 별도 스레드에서 실행 (이벤트 루프 충돌 방지)
    ros_thread = threading.Thread(target=ros2_thread, daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())  # PyQt 이벤트 루프 실행
