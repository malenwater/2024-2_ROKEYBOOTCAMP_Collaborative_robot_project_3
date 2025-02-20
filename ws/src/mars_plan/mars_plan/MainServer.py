import rclpy
from rclpy.node import Node
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from .Patro_Return_NAV import Patro_Return_NAV
from .UserCommand import UserCommand
from .GoldDetector import GoldDetector
from .obj_ctl import ModelManager

class MainServer(Node):
    """ROS2 서비스 노드 (YOLO 실행 요청을 처리)"""
    def __init__(self):
        super().__init__('MainServer')
        self.get_logger().info(f'MainServer start')
        self.ROBOT_NUMBER = 2
        self.robot = None
        self.command = None
        self.ROBOT_ORDER = []
        self.ROBOT_NAMESPACE ={}
        self.ROBOT_NODE_PATROL = {}
        self.ROBOT_NODE_PATROL_THREAD = {}
        self.ROBOT_NODE_PATROL_FLAG = {}
        self.GoldDetector = {}
        self.GoldDetector_FLAG = {}
        self.ROBOT_PATROL_WAYPOINT = { "1" : [
                                            # (1.6368083953857422, -6.828545570373535),  # 공용좌표 (상단)
                                            
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
                                                                "Patrol",
                                                                self.ROBOT_RETURN_WAYPOINT[ORDER])
            
            self.ROBOT_NODE_PATROL_THREAD[ORDER] = threading.Thread(target=self.ROBOT_NODE_PATROL[ORDER].execute_navigation_patrol)
            self.ROBOT_NODE_PATROL_THREAD[ORDER].start()
            self.GoldDetector_FLAG[ORDER] = False
            self.GoldDetector[ORDER] = GoldDetector(
                                                        self,
                                                        ORDER,
                                                        self.ROBOT_NAMESPACE[ORDER],)
        self.running = False
        
        self.command_thread = threading.Thread(target=self.ROBOT_NODE_COMMAND.process_messages, daemon=True)
        self.command_thread.start()
        
        self.worker_thread = threading.Thread(target=self.run_control, daemon=True)
        self.worker_thread.start()
        
        self.delete_ModelManager = ModelManager()
        
        self.get_logger().info(f'MainServer Setiing Done')
        self.get_logger().info(f'MainServer end')
        
    def set_robot(self,robot):
        self.robot = robot
    def set_command(self,command):
        self.command = command
        
    def run_control(self):
        self.get_logger().info(f'run_control start')
        self.running = True
        while self.running:
            robot = self.robot
            command = self.command
            # self.get_logger().info(f'run_control check')
            
            if command == "1" and self.ROBOT_NODE_PATROL_FLAG[robot] != "1": # 순찰
                self.get_logger().info(f'run_control start {command} command')
                self.ROBOT_NODE_PATROL_FLAG[robot] = "1"
                self.GoldDetector_FLAG[robot] = True
                self.get_logger().info(f'run_control check {self.GoldDetector_FLAG[robot] }')
                
            elif command == "2" and self.ROBOT_NODE_PATROL_FLAG[robot] != "2": # 귀환
                self.get_logger().info(f'run_control start {command} command')
                self.ROBOT_NODE_PATROL_FLAG[robot] = "2"
                
            elif command == "3": # 정지
                self.ROBOT_NODE_PATROL_FLAG[robot] = "0"
                self.ROBOT_NODE_PATROL[robot].cancel_goal()
                self.GoldDetector_FLAG[robot] = False
                
            elif command == "4": # 정지
                self.ROBOT_NODE_PATROL_FLAG[robot] = "3"
                self.ROBOT_NODE_PATROL[robot].send_goal(2.21324182152748108, 2.515588569641113)
                self.GoldDetector_FLAG[robot] = True
            elif command == "5": # 정지
                self.ROBOT_NODE_PATROL_FLAG[robot] = "3"
                self.ROBOT_NODE_PATROL[robot].send_goal(1.21324182152748108, 1.15588569641113)
                self.GoldDetector_FLAG[robot] = True
            self.robot =None
            self.command = None
            time.sleep(0.5)
            
        self.get_logger().info(f'run_control end')
    
    def stop_THREAD(self):
        self.get_logger().info(f'stop_THREAD start')
        
        for idx in range(1, self.ROBOT_NUMBER + 1):
            ORDER = str(idx)
            self.ROBOT_NODE_PATROL_FLAG[ORDER] = "0"  # 상태 초기화
            self.ROBOT_NODE_PATROL[ORDER].set_RUN_FLAG(False)
        
        # 스레드 종료 대기
        timeout = 5  # 최대 대기 시간 (초)
        start_time = time.time()
        
        while any(self.ROBOT_NODE_PATROL_THREAD[ORDER].is_alive() for ORDER in self.ROBOT_ORDER):
            if time.time() - start_time > timeout:
                self.get_logger().warning('Timeout reached while stopping threads')
                break
            time.sleep(0.5)  # 짧은 간격으로 확인
        
        for ORDER in self.ROBOT_ORDER:
            if self.ROBOT_NODE_PATROL_THREAD[ORDER].is_alive():
                self.ROBOT_NODE_PATROL_THREAD[ORDER].join(timeout=1)
        
        self.get_logger().info(f'stop_THREAD end')
    
    def collect_robots(self, x, y, robot):
        x = float(x)
        y = float(y)
        self.get_logger().info(f'collect_robots {x}, {y}, {robot} start')
        new_lst = [x for x in self.ROBOT_ORDER if x != "1"]
        self.get_logger().info(f'collect_robots {new_lst} new_lst')
        
        for item in new_lst:
            self.stop_NAV(item)
            self.GoldDetector_FLAG[item] = False
            self.get_logger().info(f'collect_robots {item} runnig')
            self.ROBOT_NODE_PATROL[item].send_goal(x,y)
        
        self.delete_ModelManager.delete_GOLD()
        self.get_logger().info(f'delete_GOLD')
        
        self.ROBOT_NODE_PATROL_FLAG[robot] = "2"
        self.get_logger().info(f'collect_robots {x}, {y}, {robot} end')
        
    def get_ROBOT_NODE_PATROL_FLAG(self,robot):
        return self.ROBOT_NODE_PATROL_FLAG[robot] 
    
    def get_GoldDetector_FLAG(self,robot):
        # self.get_logger().info(f'get_GoldDetector_FLAG check {self.GoldDetector_FLAG[robot] }, {robot}')
        return self.GoldDetector_FLAG[robot] 
    
    def set_GoldDetector_FLAG(self,robot, data):
        self.get_logger().info(f'get_GoldDetector_FLAG check {self.GoldDetector_FLAG[robot] }, {robot}')
        self.GoldDetector_FLAG[robot] = data
        
    def stop_NAV(self,robot):
        self.ROBOT_NODE_PATROL_FLAG[robot] = "0"
        self.ROBOT_NODE_PATROL[robot].cancel_goal()
        
def main():
    rclpy.init()
    node = MainServer()
    node.get_logger().info('MainServer main start')
    
    executor = MultiThreadedExecutor(num_threads=20)
    executor.add_node(node) 
    executor.add_node(node.ROBOT_NODE_COMMAND) 
    executor.add_node(node.delete_ModelManager) 
    for ORDER in node.ROBOT_ORDER:
        executor.add_node(node.ROBOT_NODE_PATROL[ORDER])
        executor.add_node(node.GoldDetector[ORDER])

    
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