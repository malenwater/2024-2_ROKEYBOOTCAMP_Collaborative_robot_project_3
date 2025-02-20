import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time  # 시간 지연을 위해 추가

class Robot1Navigator(Node):
    def __init__(self):
        super().__init__('robot1_navigator')
        
        self.action_client = ActionClient(self, NavigateToPose, '/tb1/navigate_to_pose')
        
        self.targets = [
            (-6.142860412597656, -1.1557469367980957),  # 1번 맵 6시
            (7.1778082847595215, -1.3470410108566284),  # 3번 맵 12시
            (6.22782564163208, 4.5692620277404785),     # 5번 맵 11시
            (2.4277842044830322, -1.5583202838897705),  # 7번 협곡 중앙
            (0.5277646780014038, -4.199518203735352)    # 9번 3시 협곡
        ]
        
        self.execute_navigation()
    
    def send_goal(self, x, y):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending goal to Robot 1: ({x}, {y})')
        
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if result_future.result().status == 4:
            self.get_logger().error("Goal was aborted")
        else:
            self.get_logger().info("Goal reached successfully")
    
    def execute_navigation(self):
        for x, y in self.targets:
            self.send_goal(x, y)


def main(args=None):
    rclpy.init(args=args)
    node = Robot1Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

