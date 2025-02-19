import rclpy
from rclpy.node import Node
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from .Patro_Return_NAV import Patro_Return_NAV
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
        self.ROBOT_NODE_PATROL_THREAD = {}
        self.ROBOT_NODE_RETURN = {}
        self.ROBOT_NODE_RETURN_THREAD = {}
        self.ROBOT_NODE_PATROL_FLAG = {}
        self.ROBOT_PATROL_WAYPOINT = { "1" : [
                                            (1.9283926486968994, 5.010260581970215),  # 6시
                                            (6.535802364349365, 4.402944087982178),  # 7시
                                            (6.9502177238464355, -1.0246244668960571),  # 9시
                                            (7.321432113647461, -5.645576000213623),  # 11시
                                            (1.6368083953857422, -6.828545570373535),  # 공용좌표 (상단)
                                            (2.022683620452881, -1.4855573177337646)  # 공용좌표 (중앙)
                                        ],
                                 "2" : [
                                            (-0.11324182152748108, 4.215588569641113),  # 6시
                                            (-2.5168874263763428, 1.9160888195037842),  # 5시
                                            (-4.162641525268555, -4.875141143798828),  # 3시
                                            (-3.0764317512512207, -7.6846489906311035),  # 1시
                                            (1.6368083953857422, -6.828545570373535),  # 공용좌표 (상단)
                                            (2.022683620452881, -1.4855573177337646)  # 공용좌표 (중앙)
                                        ],}
        
        self.ROBOT_RETURN_WAYPOINT = { "1" : [
                                            (1.5, 4.0),  # 6시
                                        ],
                                 "2" : [
                                            (1.5, 5.0),  # 6시
                                        ],}
        
        self.ROBOT_NODE_COMMAND = UserCommand(self)
        
        for idx in range(1,self.ROBOT_NUMBER + 1):
            ORDER = str(idx)
            self.ROBOT_ORDER.append(ORDER)
            self.ROBOT_NAMESPACE[ORDER] = "tb" + ORDER
            self.ROBOT_NODE_PATROL_FLAG[ORDER] = "0"
            self.ROBOT_NODE_PATROL[ORDER] = Patro_Return_NAV(self.ROBOT_PATROL_WAYPOINT[ORDER],
                                                                self.ROBOT_NAMESPACE[ORDER],
                                                                ORDER,
                                                                self,
                                                                "Patrol")
            self.ROBOT_NODE_PATROL_THREAD[ORDER] = threading.Thread(target=self.ROBOT_NODE_PATROL[ORDER].execute_navigation_patrol)
            self.ROBOT_NODE_PATROL_THREAD[ORDER].start()
            self.ROBOT_NODE_RETURN[ORDER] = Patro_Return_NAV(self.ROBOT_RETURN_WAYPOINT[ORDER],
                                                                self.ROBOT_NAMESPACE[ORDER],
                                                                ORDER,
                                                                self,
                                                                "return")
            self.ROBOT_NODE_RETURN_THREAD[ORDER] = threading.Thread(target=self.ROBOT_NODE_RETURN[ORDER].execute_navigation_return)
            self.ROBOT_NODE_RETURN_THREAD[ORDER].start()
            
        self.get_logger().info(f'MainServer Setiing Done')
        self.get_logger().info(f'MainServer end')
        
    def run_control(self, robot, command):
        self.get_logger().info(f'run_control start')
        if command == "1" and self.ROBOT_NODE_PATROL_FLAG[robot] != "1": # 순찰
            self.get_logger().info(f'run_control start {command}')
            self.ROBOT_NODE_PATROL_FLAG[robot] = "1"
            
        elif command == "2" and self.ROBOT_NODE_PATROL_FLAG[robot] != "2": # 귀환
            self.get_logger().info(f'run_control start {command}')
            self.ROBOT_NODE_PATROL_FLAG[robot] = "0"
            time.sleep(2)
            self.ROBOT_NODE_PATROL_FLAG[robot] = "2"
        elif command == "3": # 정지
            self.ROBOT_NODE_PATROL_FLAG[robot] = "0"
        else:
            self.get_logger().info(f'User insert wrong command')
        self.get_logger().info(f'run_control stop')
    
    def stop_THREAD(self):
        self.get_logger().info(f'stop_THREAD start')
        
        for idx in range(1, self.ROBOT_NUMBER + 1):
            ORDER = str(idx)
            self.ROBOT_NODE_PATROL_FLAG[ORDER] = "0"  # 상태 초기화
            self.ROBOT_NODE_PATROL[ORDER].set_RUN_FLAG(False)
            # self.ROBOT_NODE_RETURN[ORDER].set_RUN_FLAG(False)
        
        # 스레드 종료 대기
        timeout = 5  # 최대 대기 시간 (초)
        start_time = time.time()
        
        while any(self.ROBOT_NODE_PATROL_THREAD[ORDER].is_alive() or self.ROBOT_NODE_RETURN_THREAD[ORDER].is_alive() for ORDER in self.ROBOT_ORDER):
            if time.time() - start_time > timeout:
                self.get_logger().warning('Timeout reached while stopping threads')
                break
            time.sleep(0.5)  # 짧은 간격으로 확인
        
        for ORDER in self.ROBOT_ORDER:
            if self.ROBOT_NODE_PATROL_THREAD[ORDER].is_alive():
                self.ROBOT_NODE_PATROL_THREAD[ORDER].join(timeout=1)
            # if self.ROBOT_NODE_RETURN_THREAD[ORDER].is_alive():
            #     self.ROBOT_NODE_RETURN_THREAD[ORDER].join(timeout=1)
        
        self.get_logger().info(f'stop_THREAD end')
        
    def get_ROBOT_NODE_PATROL_FLAG(self,robot):
        return self.ROBOT_NODE_PATROL_FLAG[robot] 
   
def main():
    rclpy.init()
    node = MainServer()
    node.get_logger().info('MainServer main start')
    
    executor = MultiThreadedExecutor()
    executor.add_node(node) 
    executor.add_node(node.ROBOT_NODE_COMMAND) 
    for ORDER in node.ROBOT_ORDER:
        executor.add_node(node.ROBOT_NODE_PATROL[ORDER])
        # executor.add_node(node.ROBOT_NODE_RETURN[ORDER])

    
    try:
        executor.spin()  # ROS 2 이벤트 루프 실행
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MainServer...")
    finally:
        node.stop_THREAD()  # 스레드 정리
        node.destroy_node()
        if rclpy.ok():  # 이미 종료되지 않았다면 shutdown 수행
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()