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
            (1.9283926486968994, 5.010260581970215),  # 6시
            (6.535802364349365, 4.402944087982178),  # 7시
            (6.9502177238464355, -1.0246244668960571),  # 9시
            (7.321432113647461, -5.645576000213623),  # 11시
            (1.6368083953857422, -6.828545570373535),  # 공용좌표 (상단)
            (2.022683620452881, -1.4855573177337646)  # 공용좌표 (중앙)
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

