'''
이강태
<GUI 버튼>

1.로봇1 버튼:
로봇1의 현재 위치를 가져옴
로봇2에게 로봇1의 위치를 전송
로봇2가 로봇1의 위치로 이동

2.로봇2 버튼:
로봇2의 현재 위치를 가져옴
로봇1에게 로봇2의 위치를 전송
로봇1이 로봇2의 위치로 이동

3.복귀 버튼

로봇1과 로봇2 모두 미리 정한 시작점(home)으로 복귀

'''

import rclpy
from rclpy.node import Node
import tkinter as tk
from geometry_msgs.msg import PoseStamped
import threading

# 로봇 위치 저장 변수
robot1_position = None
robot2_position = None
home_position = {"x": 0.0, "y": 0.0, "z": 0.0}  # 홈 위치 (임의로 지정)

class RobotControlNode(Node):
    def __init__(self):
        super().__init__("robot_gui_node")

        # 로봇1과 로봇2의 위치 구독
        self.robot1_subscriber = self.create_subscription(PoseStamped, "/robot1/pose", self.robot1_pose_callback, 10)
        self.robot2_subscriber = self.create_subscription(PoseStamped, "/robot2/pose", self.robot2_pose_callback, 10)
        
        # 로봇 간 위치 전송
        self.robot1_to_robot2_publisher = self.create_publisher(PoseStamped, "/robot2/goal_pose", 10)
        self.robot2_to_robot1_publisher = self.create_publisher(PoseStamped, "/robot1/goal_pose", 10)
        
        # 복귀 명령 발행
        self.robot1_home_publisher = self.create_publisher(PoseStamped, "/robot1/goal_pose", 10)
        self.robot2_home_publisher = self.create_publisher(PoseStamped, "/robot2/goal_pose", 10)

    def robot1_pose_callback(self, msg):
        global robot1_position
        robot1_position = {"x": msg.pose.position.x, "y": msg.pose.position.y, "z": msg.pose.position.z}
        self.get_logger().info(f"로봇1 위치 업데이트: {robot1_position}")

    def robot2_pose_callback(self, msg):
        global robot2_position
        robot2_position = {"x": msg.pose.position.x, "y": msg.pose.position.y, "z": msg.pose.position.z}
        self.get_logger().info(f"로봇2 위치 업데이트: {robot2_position}")

    def send_robot1_position_to_robot2(self):
        if robot1_position:
            msg = PoseStamped()
            msg.pose.position.x = robot1_position["x"]
            msg.pose.position.y = robot1_position["y"]
            msg.pose.position.z = robot1_position["z"]
            self.robot1_to_robot2_publisher.publish(msg)
            self.get_logger().info("로봇2가 로봇1의 위치로 이동 중...")
        else:
            self.get_logger().warn("로봇1 위치를 아직 받지 못함.")

    def send_robot2_position_to_robot1(self):
        if robot2_position:
            msg = PoseStamped()
            msg.pose.position.x = robot2_position["x"]
            msg.pose.position.y = robot2_position["y"]
            msg.pose.position.z = robot2_position["z"]
            self.robot2_to_robot1_publisher.publish(msg)
            self.get_logger().info("로봇1이 로봇2의 위치로 이동 중...")
        else:
            self.get_logger().warn("로봇2 위치를 아직 받지 못함.")

    def send_robots_home(self):
        home_msg = PoseStamped()
        home_msg.pose.position.x = home_position["x"]
        home_msg.pose.position.y = home_position["y"]
        home_msg.pose.position.z = home_position["z"]

        self.robot1_home_publisher.publish(home_msg)
        self.robot2_home_publisher.publish(home_msg)
        self.get_logger().info("로봇1, 로봇2 복귀 중...")

class RobotGUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("ROS 2 Robot Controller")

        self.robot1_btn = tk.Button(self.root, text="로봇1 → 로봇2 이동", command=self.send_robot1_to_robot2)
        self.robot1_btn.pack(pady=10)

        self.robot2_btn = tk.Button(self.root, text="로봇2 → 로봇1 이동", command=self.send_robot2_to_robot1)
        self.robot2_btn.pack(pady=10)

        self.home_btn = tk.Button(self.root, text="복귀", command=self.send_robots_home)
        self.home_btn.pack(pady=10)

        self.status_label = tk.Label(self.root, text="로봇 상태: 대기 중")
        self.status_label.pack(pady=10)

    def send_robot1_to_robot2(self):
        self.node.send_robot1_position_to_robot2()
        self.status_label.config(text="로봇2가 로봇1 위치로 이동 중...")

    def send_robot2_to_robot1(self):
        self.node.send_robot2_position_to_robot1()
        self.status_label.config(text="로봇1이 로봇2 위치로 이동 중...")

    def send_robots_home(self):
        self.node.send_robots_home()
        self.status_label.config(text="로봇1, 로봇2 복귀 중...")

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = RobotControlNode()
    gui = RobotGUI(node)
    gui_thread = threading.Thread(target=gui.run, daemon=True)
    gui_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
