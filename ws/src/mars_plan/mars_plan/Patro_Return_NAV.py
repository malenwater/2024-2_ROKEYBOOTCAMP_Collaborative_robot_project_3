import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time  # 시간 지연을 위해 추가

class Patro_Return_NAV(Node):
    """ROS2 서비스 노드 (Patro_Return_NAV 실행 요청을 처리)"""
    def __init__(self,target, NAMESPACE, ORDER, MainServer,NAME):
        super().__init__(NAME + NAMESPACE)
        self.get_logger().info(f'{NAME} {NAMESPACE} start')
        self.RUN_FLAG = False
        self.NAME = NAME
        self.NAMESPACE = NAMESPACE
        self.ORDER = ORDER
        self.action_client = ActionClient(self, NavigateToPose, '/'+ NAMESPACE + '/navigate_to_pose')
        self.MainServer = MainServer
        self.targets = target
        
        self.get_logger().info(f'{self.NAME} {self.NAMESPACE} end')
    
    def send_goal(self, x, y):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{self.get_name()} Action server not available!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal to Robot: ({x}, {y})')

        # 비동기 요청 -> Future 완료될 때까지 대기
        goal_future = self.action_client.send_goal_async(goal_msg)
        while not goal_future.done():
            time.sleep(0.1)  # CPU 과부하 방지

        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return
        
        # 결과 기다리기 (로봇이 목표에 도착할 때까지 대기)
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)  # CPU 과부하 방지

        result = result_future.result()
        if result.status == 4:
            self.get_logger().error("Goal was aborted")
        else:
            self.get_logger().info("Goal reached successfully")

    def execute_navigation_return(self):
        self.RUN_FLAG = True
        self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_return THREAD start')
        while self.RUN_FLAG:
            # self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_return THREAD mode check')
            if self.MainServer.get_ROBOT_NODE_PATROL_FLAG(self.ORDER) == "2":
                self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_return THREAD 2 mode start')
                self.send_goal(self.targets[0][0], self.targets[0][1])
                self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_return THREAD 2 mode end')
            time.sleep(1)
        self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_return THREAD end')
        
    def execute_navigation_patrol(self):
        self.RUN_FLAG = True
        self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD start')
        while self.RUN_FLAG:
            # self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD mode check')
            if self.MainServer.get_ROBOT_NODE_PATROL_FLAG(self.ORDER) == "1":
                self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD 1 mode start')
                idx = 0
                waypoint_len = len(self.targets)
                while self.MainServer.get_ROBOT_NODE_PATROL_FLAG(self.ORDER) == "1":
                    x, y = self.targets[idx][0], self.targets[idx][1]
                    self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD 1 mode running {x} {y}')
                    self.send_goal(x, y)
                    idx = (idx + 1) % waypoint_len  # 순환하도록 변경
                self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD 1 mode end')
                
            time.sleep(1)
        self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD end')
        
    def set_RUN_FLAG(self,DATA):
        self.get_logger().info(f'{self.NAME} {self.NAMESPACE} set_RUN_FLAG {DATA} change')
        self.RUN_FLAG = DATA
        
def main(args=None):
    rclpy.init(args=args)
    node = Patro_Return_NAV([
                                (-6.142860412597656, -1.1557469367980957),  # 1번 맵 6시
                                (7.1778082847595215, -1.3470410108566284),  # 3번 맵 12시
                                (6.22782564163208, 4.5692620277404785),     # 5번 맵 11시
                                (2.4277842044830322, -1.5583202838897705),  # 7번 협곡 중앙
                                (0.5277646780014038, -4.199518203735352)    # 9번 3시 협곡
                            ],
                             "/tb1",
                             "1",
                             None,
                             "Patrol")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

