import rclpy
from rclpy.node import Node
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from .Patrol_Return_NAV import Patrol_Return_NAV
from .UserCommand import UserCommand

class MainServer(Node):
    """ROS2 서비스 노드 (YOLO 실행 요청을 처리)"""
    def __init__(self):
        super().__init__('MainServer')
        self.get_logger().info(f'MainServer start')
        self.ROBOT_NUMBER = 2
        self.ROBOT_ORDER = []
        self.ROBOT_NAMESPACE ={}
        self.ROBOT_NODE_PATROL = {}
        self.ROBOT_PATROL_WAYPOINT = { "1" : [
                                            (-6.142860412597656, -1.1557469367980957),  # 1번 맵 6시
                                            (7.1778082847595215, -1.3470410108566284),  # 3번 맵 12시
                                            (6.22782564163208, 4.5692620277404785),     # 5번 맵 11시
                                            (2.4277842044830322, -1.5583202838897705),  # 7번 협곡 중앙
                                            (0.5277646780014038, -4.199518203735352)    # 9번 3시 협곡
                                        ],
                                 "2" : [
                                            (1.6889194250106812, 6.10117769241333),  # 2번 맵 9시
                                            (1.9527602195739746, -7.316145420074463), # 4번 맵 3시
                                            (7.441680431365967, -6.1012115478515625), # 6번 맵 1시
                                            (4.063904285430908, -1.3998534679412842), # 8번 12시 협곡
                                            (1.6361191272735596, 0.13205400109291077) # 10번 9시 협곡 입구
                                        ],}
        self.ROBOT_NODE_COMMAND = UserCommand(self)
        
        for idx in range(1,self.ROBOT_NUMBER + 1):
            ORDER = str(idx)
            self.ROBOT_ORDER.append(ORDER)
            self.ROBOT_NAMESPACE[ORDER] = "tb" + ORDER
            self.ROBOT_NODE_PATROL[ORDER] = Patrol_Return_NAV(self.ROBOT_PATROL_WAYPOINT[ORDER],
                                                                self.ROBOT_NAMESPACE[ORDER])
            
        self.get_logger().info(f'MainServer Setiing Done')
        self.get_logger().info(f'MainServer end')
        
    def run_control(self, robot, command):
        self.get_logger().info(f'run_control start')
        if command == "1": # 순찰
            self.ROBOT_NODE_PATROL[robot].execute_navigation()
        elif command == "2": # 정지
            pass
        elif command == "3": # 귀환
            pass
        else:
            self.get_logger().info(f'User insert wrong command')
        self.get_logger().info(f'run_control stop')

   
def main():
    rclpy.init()
    node = MainServer()
    node.get_logger().info('MainServer main start')
    
    executor = MultiThreadedExecutor()
    executor.add_node(node) 
    executor.add_node(node.ROBOT_NODE_COMMAND) 
    for ORDER in node.ROBOT_ORDER:
        executor.add_node(node.ROBOT_NODE_PATROL[ORDER])

    
    try:
        executor.spin()  # ROS 2 이벤트 루프 시작
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 쓰레드와 노드 종료 처리
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()