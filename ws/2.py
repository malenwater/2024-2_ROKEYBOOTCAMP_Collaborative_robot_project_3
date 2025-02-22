import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time  # 시간 지연을 위해 추가

class Robot2Navigator(Node):
    def __init__(self):
        super().__init__('robot2_navigator')
        
        self.action_client = ActionClient(self, NavigateToPose, '/tb2/navigate_to_pose')
        
        self.targets = [
            (1.6889194250106812, 6.10117769241333),  # 2번 맵 9시
            (1.9527602195739746, -7.316145420074463), # 4번 맵 3시
            (7.441680431365967, -6.1012115478515625), # 6번 맵 1시
            (4.063904285430908, -1.3998534679412842), # 8번 12시 협곡
            (1.6361191272735596, 0.13205400109291077) # 10번 9시 협곡 입구
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
        
        self.get_logger().info(f'Sending goal to Robot 2: ({x}, {y})')
        
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
    node = Robot2Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

