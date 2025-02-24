import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.task import Future

class Robot2Navigator(Node):
    def __init__(self):
        super().__init__('gold_detector_rb2')
        self.subscription = self.create_subscription(
            PoseStamped, '/robot1/position', self.position_callback, 10)
        
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.latest_position = None  # 최신 위치 저장

    def position_callback(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f"📍 로봇1 위치 수신: x={x}, y={y}")

        # 위치가 정상적인 값인지 확인
        if x == 0.0 and y == 0.0:
            self.get_logger().warn("⚠️ 로봇1의 위치가 (0.0, 0.0)입니다. 이동 요청 생략.")
            return

        # 같은 위치가 연속적으로 퍼블리시될 경우 중복 요청 방지
        if self.latest_position and (self.latest_position.pose.position.x == x and self.latest_position.pose.position.y == y):
            self.get_logger().info("🔄 동일한 위치 수신, 이동 요청 생략")
            return

        self.latest_position = msg  # 최신 위치 업데이트

        # Nav2 목표 설정
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"  # 고정된 좌표계를 사용
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = msg.pose  # 로봇1의 위치를 목표로 설정
        
        self.get_logger().info("🚀 로봇2가 로봇1 위치로 이동 시작!")
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        result = future.result()
        if not result.accepted:
            self.get_logger().warn("⚠️ Nav2 목표가 거부됨. 다시 시도 필요.")
        else:
            self.get_logger().info("✅ Nav2 목표가 성공적으로 수락됨!")


def main(args=None):
    rclpy.init(args=args)
    node = Robot2Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
