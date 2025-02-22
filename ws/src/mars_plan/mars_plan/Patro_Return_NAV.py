import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time  # 시간 지연을 위해 추가

'''
#  ROS2 기반의 로봇 순찰 및 귀환 시스템을 관리하는 노드

ROS2 기반의 로봇 순찰 및 귀환을 제어하는 노드.
Nav2 액션 클라이언트를 사용하여 로봇을 특정 위치로 이동하도록 함.
순찰 모드(1)와 귀환 모드(2)를 지원.




# 해당 로봇의 순찰 경로,
            self.ROBOT_NODE_PATROL[ORDER] = Patro_Return_NAV(self.ROBOT_PATROL_WAYPOINT[ORDER],  => waypoint들이 target
                                                                self.ROBOT_NAMESPACE[ORDER],     => Namespace
                                                                ORDER,
                                                                self,                            => mainserver
                                                                "Patrol",                        => Name
                                                                self.ROBOT_RETURN_WAYPOINT[ORDER] => return_target
                                                                )
'''

class Patro_Return_NAV(Node):
    """ROS2 서비스 노드 (Patro_Return_NAV 실행 요청을 처리)"""
    def __init__(self,target, NAMESPACE, ORDER, MainServer,NAME,return_target):
        super().__init__(NAME + NAMESPACE)
        self.get_logger().info(f'{NAME} {NAMESPACE} start')
        self.RUN_FLAG = False
        self.NAME = NAME
        self.NAMESPACE = NAMESPACE
        self.ORDER = ORDER
        
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/'+ NAMESPACE + '/navigate_to_pose')
        
        self.MainServer = MainServer
        self.targets = target
        self.goal_handle = None
        self.return_target = return_target
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
            time.sleep(0.5)  # CPU 과부하 방지

        self.goal_handle = goal_future.result()
        if not self.goal_handle or not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return
        
        # 결과 기다리기 (로봇이 목표에 도착할 때까지 대기)
        result_future = self.goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.5)  # CPU 과부하 방지

        result = result_future.result()
        if result.status == 4:
            self.get_logger().error("Goal was aborted")
        else:
            self.get_logger().info("Goal reached successfully")
            
        self.goal_handle = None
        
    def cancel_goal(self):
        """목표 취소 요청 후 동기적으로 대기"""
        self.get_logger().info(f"{self.NAMESPACE} Attempting to cancel goal...")

        if not self.goal_handle:
            self.get_logger().error("No goal handle available to cancel")
            return
        
        cancel_future = self.goal_handle.cancel_goal_async()

        # 🚀 while 문으로 대기, 너무 많이 돌지 않도록 sleep 추가
        timeout = 3  # 최대 3초 대기
        start_time = time.time()

        while not cancel_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout while waiting for goal cancellation")
                return
            time.sleep(0.1)  # CPU 점유율 낮추기 위해 0.1초씩 대기

        cancel_response = cancel_future.result()
        
        if cancel_response.return_code == 0:
            self.get_logger().info("Goal successfully cancelled")
        else:
            self.get_logger().error(f"Failed to cancel goal, return code: {cancel_response.return_code}")

        self.goal_handle = None


        
    def execute_navigation_patrol(self):
        self.RUN_FLAG = True
        self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD start')
        while self.RUN_FLAG:
            self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_patrol THREAD mode check')
            # order부분에는 로봇번호_ 예를들어 1,2가 들어간다
            # 1. 순찰 
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
                
            # 2. 귀환
            elif self.MainServer.get_ROBOT_NODE_PATROL_FLAG(self.ORDER) == "2":
                self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_return THREAD 2 mode start')
                self.send_goal(self.return_target[0][0], self.return_target[0][1])
                self.get_logger().info(f'{self.NAME} {self.NAMESPACE} execute_navigation_return THREAD 2 mode end')
                
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
                            ], # target_광물 위치
                             "/tb1",# namespace
                             "1", # order
                             None, # Mainserver
                             "Patrol")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

