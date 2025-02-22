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
    """MainServer 노드 """
    def __init__(self):
        super().__init__('MainServer')
        self.get_logger().info(f'MainServer start')
        self.ROBOT_NUMBER = 2
        self.robot = None
        self.command = None
        # 로봇 id 저장하는 리스트 _ 로봇 리스트 관리용
        # '1', '2'
        self.ROBOT_ORDER = []

        # 각 로봇의 네임스페이스 저장
        #  self.ROBOT_NAMESPACE['1'] = "tb1"
        self.ROBOT_NAMESPACE ={}

        # 각 로봇의 순찰 객체를 저장하는 딕셔너리
        #  PATRO_Return_NAV클래스 사용하여, 로봇별 순찰을 진행시킴.
        self.ROBOT_NODE_PATROL = {}

        # 각 로봇의 순찰을 수행하는 **스레드(별도 실행 흐름)**를 저장하는 딕셔너리
        self.ROBOT_NODE_PATROL_THREAD = {}

        # 각 로봇의 순찰 상태를 저장하는 딕셔너리
        #  0,정지 1,순찰중 2,귀환중 , 으로 사용 가능
        self.ROBOT_NODE_PATROL_FLAG = {}

        # 각 로봇의 골드 감지를 수행하는 객체를 저장하는 딕셔너리
        self.GoldDetector = {}

        #  각 로봇의 골드 감지 기능이 활성화되었는지 여부를 저장하는 딕셔너리
        self.GoldDetector_FLAG = {}
        
        # 순찰 waypoint
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
        # 귀환 waypoint
        self.ROBOT_RETURN_WAYPOINT = { "1" : [
                                            (1.5, 4.0),  # 6시
                                        ],
                                 "2" : [
                                            (1.5, 5.0),  # 6시
                                        ],}
        

        # Mainserver의 객체_self을 전달함,
        # usercommand는 mainswerver와 연결 
        #  -> 명령을 수신하면, 해당 명령을 Mainserver의 변수에 저장
        self.ROBOT_NODE_COMMAND = UserCommand(self)
        
        for idx in range(1,self.ROBOT_NUMBER + 1):
            # ORDER 는 로봇1, 로봇2의 1,2를 의미함, 
            ORDER = str(idx) #'1', '2'
            self.ROBOT_ORDER.append(ORDER)
            # 각 로봇의 네임스페이스 설정 => tb1, tb2
            self.ROBOT_NAMESPACE[ORDER] = "tb" + ORDER
            # 순찰 상태, 초기상태는 0으로 대기상태로 설정해놓음 
            # 딕셔너리 형태로, 키값에는 로봇번호 / 벨류에는 명령어플래그 번호가 들어간다./ 초기에는 0으로 설정.
            self.ROBOT_NODE_PATROL_FLAG[ORDER] = "0"
            # 해당 로봇의 순찰 경로,
            self.ROBOT_NODE_PATROL[ORDER] = Patro_Return_NAV(self.ROBOT_PATROL_WAYPOINT[ORDER],
                                                                self.ROBOT_NAMESPACE[ORDER],
                                                                ORDER,
                                                                self,
                                                                "Patrol",
                                                                self.ROBOT_RETURN_WAYPOINT[ORDER])
            # 해당 로봇의 귀환 경로,
            self.ROBOT_NODE_PATROL_THREAD[ORDER] = threading.Thread(target=self.ROBOT_NODE_PATROL[ORDER].execute_navigation_patrol)
            # 각 로봇의 순찰을 실행
            self.ROBOT_NODE_PATROL_THREAD[ORDER].start()
            # 초기에는 골드 감지를 비활성화 
            self.GoldDetector_FLAG[ORDER] = False
            # 각 로봇의 골드 감지 기능을 활성화 
            self.GoldDetector[ORDER] = GoldDetector(
                                                        self,
                                                        ORDER,
                                                        self.ROBOT_NAMESPACE[ORDER],)
        # 초기 제어루프 실행 여부를 결정하는 플래그
        #  -> true가 되면 run_control함수가 실행된다.
        self.running = False
        
        self.command_thread = threading.Thread(target=self.ROBOT_NODE_COMMAND.process_messages, daemon=True)
        self.command_thread.start()
        
        # run_control 함수 실행을 위한 제어 스레드를 시작
        #  run_control 은 로봇 상태를 주기적으로 체크하고 행동을 결정하는 역할을 함,
        self.worker_thread = threading.Thread(target=self.run_control, daemon=True)
        self.worker_thread.start()
        
        # ModelManager객체를 생성하여, 모델 삭제 기능을 추가함.
        self.delete_ModelManager = ModelManager()
        
        self.get_logger().info(f'MainServer Setiing Done')
        self.get_logger().info(f'MainServer end')
        
    def set_robot(self,robot):
        self.robot = robot
    def set_command(self,command):
        self.command = command
        
    # 로봇 동작 모드 설정 
    # 로봇 행동 관리 -> self.robot, self.command에 따라 로봇 동작이 결정된다.
    def run_control(self):
        self.get_logger().info(f'run_control start')
        self.running = True
        while self.running:
            robot = self.robot
            command = self.command
            # self.get_logger().info(f'run_control check')
            
            # 1.순찰
            if command == "1" and self.ROBOT_NODE_PATROL_FLAG[robot] != "1": # 순찰
                self.get_logger().info(f'run_control start {command} command')
                self.ROBOT_NODE_PATROL_FLAG[robot] = "1"
                # goal detector 활성화
                self.GoldDetector_FLAG[robot] = True
                self.get_logger().info(f'run_control check {self.GoldDetector_FLAG[robot] }')

            # 2. 귀환  
            elif command == "2" and self.ROBOT_NODE_PATROL_FLAG[robot] != "2": # 귀환
                self.get_logger().info(f'run_control start {command} command')
                self.ROBOT_NODE_PATROL_FLAG[robot] = "2"
            # 3. 정지   
            elif command == "3": # 정지
                self.ROBOT_NODE_PATROL_FLAG[robot] = "0"
                self.ROBOT_NODE_PATROL[robot].cancel_goal()
                # goal detector 비활성화
                self.GoldDetector_FLAG[robot] = False
                
            elif command == "4": # 궤도 폭격
                self.ROBOT_NODE_PATROL_FLAG[robot] = "4"
                self.delete_ModelManager.delete_GOLD()
                
            elif command == "5": # 광물 생성
                self.ROBOT_NODE_PATROL_FLAG[robot] = "5"
                self.delete_ModelManager.make_GOLD()
                
            elif command == "6": # 광물 회수
                self.ROBOT_NODE_PATROL_FLAG[robot] = "6"
                self.delete_ModelManager.collect_minerals()
            
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
    
    # collect_robot
    # 순찰 중, 금색 감지하면, 모든 로봇을 같은 위치로 이동
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
        
        # self.delete_ModelManager.delete_GOLD()
        # self.get_logger().info(f'delete_GOLD')
        
        # for item in self.ROBOT_ORDER:
        #     self.ROBOT_NODE_PATROL_FLAG[item] = "2"
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
    # 20개의 병렬 스레드를 생성
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