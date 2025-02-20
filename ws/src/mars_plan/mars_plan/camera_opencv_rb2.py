# 이강태 250220

# 현재 기능은, 로봇1의 현재 위치를 받아, 이동
# 로봇1의 위치를 구독하여 Nav2를 사용해 해당 위치로 이동.
# 로봇2 입장


# 로봇1의 opencv박스로 정지 -> 로봇2가 로봇1 위치로 이동하면,
#  다음으로 로봇1,로봇2에 같은 세팅을 해놓아야 함.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class Robot2Navigator(Node):
    def __init__(self):
        super().__init__('gold_detector_rb2')
        self.subscription = self.create_subscription(
            PoseStamped, '/robot1/position', self.position_callback, 10)
        
        # ActionClient을 통해 Nav2
        # ActionClient: ROS2에서 액션을 실행하는 클라이언트 객체.
        # NavigateToPose: Nav2에서 목표 위치로 이동하기 위한 액션 메시지.
        # Nav2는 기본적으로 액션 기반 네비게이션 시스템을 사용하며, 이를 통해 로봇이 특정 좌표로 이동하도록 요청
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose, # nav2 action 메세지
            'navigate_to_pose' # nav2에서 제공하는 기본 액션 서버 이름
            # 액션 서버는 로봇의 경로 계획(Planning), 경로 실행(Execution), 
            #  충돌 회피(Collision Avoidance) 등을 포함한 전체적인 네비게이션 제어를 담당
            #  해당 서버가 사람대신에 cmd_vel을 퍼블리시하여 이동
        )

    def position_callback(self, msg):
        self.get_logger().info(f"📍 로봇1 위치 수신: x={msg.pose.position.x}, y={msg.pose.position.y}")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        self.get_logger().info("🚀 로봇2가 로봇1 위치로 이동 시작!")
        # nav2에 goal_msg을 비동기로 보냄
        self.nav_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Robot2Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

