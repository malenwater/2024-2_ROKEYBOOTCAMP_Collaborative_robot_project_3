import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import time
import threading  # 스레드 사용

class NavCancelExample(Node):
    def __init__(self):
        super().__init__('nav_cancel_example')
        self.action_client = ActionClient(self, NavigateToPose, '/tb2/navigate_to_pose')
        self.goal_handle = None  # Goal 핸들 저장

    def send_goal(self, x, y):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal to Robot: ({x}, {y})')

        # 목표를 비동기적으로 전송
        goal_future = self.action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response_callback)

        # 3초 후 자동으로 취소하는 스레드 실행
        threading.Thread(target=self.cancel_after_delay, args=(3,), daemon=True).start()

    def goal_response_callback(self, future):
        """Goal 전송 후 응답 콜백"""
        self.goal_handle = future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
        else:
            self.get_logger().info("Goal successfully sent!")

    def cancel_after_delay(self, delay):
        """지정한 시간 후 목표 취소"""
        time.sleep(delay)
        self.get_logger().info("Attempting to cancel goal...")

        if not self.goal_handle:
            self.get_logger().error("No goal handle available to cancel")
            return
        
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """취소 요청 후 응답 콜백"""
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info("Goal successfully cancelled")
        else:
            self.get_logger().error(f"Failed to cancel goal, return code: {cancel_response.return_code}")

def main(args=None):
    rclpy.init(args=args)
    node = NavCancelExample()
    node.send_goal(6.9502177238464355, -1.0246244668960571)  # 목표 좌표 설정 (x=2.0, y=3.0)
    
    # ROS2 노드 실행 (콜백 처리)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
