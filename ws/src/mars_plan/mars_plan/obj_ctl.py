import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
import time
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('mars_plan')
import os

class ModelManager(Node):
    def __init__(self):
        super().__init__('model_manager')
        # SpawnEntity와 DeleteEntity 클라이언트 생성
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        
        # 서비스가 준비될 때까지 대기
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn 서비스가 준비되지 않았습니다. 다시 기다립니다...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete 서비스가 준비되지 않았습니다. 다시 기다립니다...')

    def delete_GOLD(self):
        
        urdf_file_path = os.path.join(pkg_share, 'urdf', 'boom.urdf')  # SDF 파일 경로
        self.get_logger().info(f'{urdf_file_path} ')
        
        # boom 모델 생성
        self.spawn_model('boom', urdf_file_path, 5.0, -2.0, 0.0)  # boom 모델 생성

        # 3초 후에 gold 모델 삭제
        time.sleep(2)
        self.delete_model('gold')

        # 3초 후에 boom 모델 삭제
        time.sleep(2)
        self.delete_model('boom')
        
    def spawn_model(self, model_name, urdf_file_path, x, y, z):
        model_pose = Pose()
        model_pose.position.x = x
        model_pose.position.y = y
        model_pose.position.z = z
        model_pose.orientation.w = 1.0  # 회전값 설정

        # SpawnEntity 요청 생성
        request = SpawnEntity.Request()
        request.name = model_name
        try:
            with open(urdf_file_path, 'r') as file:
                request.xml = file.read()  # URDF 파일 내용 읽기
        except FileNotFoundError:
            self.get_logger().error(f"URDF 파일을 찾을 수 없습니다: {urdf_file_path}")
            return

        # 'initial_pose' 속성으로 모델 위치 설정
        request.initial_pose = model_pose  # 'pose' 대신 'initial_pose' 사용
        request.reference_frame = "world"  # 월드 기준으로 모델 배치

        # 모델 생성 요청
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'{model_name} 모델이 성공적으로 생성되었습니다.')
        else:
            self.get_logger().error(f'{model_name} 모델 생성에 실패했습니다.')

    def delete_model(self, model_name):
        # DeleteEntity 요청 생성
        request = DeleteEntity.Request()
        request.name = model_name

        # 모델 삭제 요청
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'{model_name} 모델이 성공적으로 삭제되었습니다.')
        else:
            self.get_logger().error(f'{model_name} 모델 삭제에 실패했습니다.')

def main(args=None):
    rclpy.init(args=args)
    manager = ModelManager()
    manager.delete_GOLD()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# import rclpy
# from gazebo_msgs.srv import SpawnEntity
# from geometry_msgs.msg import Pose
# from rclpy.node import Node

# class ModelSpawner(Node):
#     def __init__(self):
#         super().__init__('model_spawner')
#         self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
#         # 서비스가 준비될 때까지 대기
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('서비스가 준비되지 않았습니다. 다시 기다립니다...')
            
#         urdf_file_path = os.path.join(pkg_share, 'urdf', 'boom.sdf')  # SDF 파일 경로
#         # URDF 파일 경로와 모델 이름 지정
#         # urdf_file_path = '/home/jsy/mars_mv/boom.urdf'  # URDF 파일 경로를 알맞게 수정하세요
#         model_name = 'boom'
        
#         # 모델의 위치와 회전 설정
#         model_pose = Pose()
#         model_pose.position.x = 0.0
#         model_pose.position.y = 0.0
#         model_pose.position.z = 0.0  # 바닥 위에 위치하도록 설정
#         model_pose.orientation.w = 1.0  # 기본 회전값 (회전 없음)
        
#         # SpawnEntity 요청 생성
#         request = SpawnEntity.Request()
#         request.name = model_name
#         try:
#             with open(urdf_file_path, 'r') as file:
#                 request.xml = file.read()  # URDF 파일 내용을 읽기
#         except FileNotFoundError:
#             self.get_logger().error(f"URDF 파일을 찾을 수 없습니다: {urdf_file_path}")
#             return

#         # 'initial_pose' 속성으로 모델 위치 설정
#         request.initial_pose = model_pose  # 'pose' 대신 'initial_pose' 사용
#         request.reference_frame = "world"  # 모델을 월드 기준으로 배치
        
#         # 모델 스폰 서비스 호출
#         future = self.client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)
#         if future.result() is not None:
#             self.get_logger().info('모델이 성공적으로 생성되었습니다.')
#         else:
#             self.get_logger().error('모델 생성에 실패했습니다.')

# def main(args=None):
#     rclpy.init(args=args)
#     spawner = ModelSpawner()
#     rclpy.spin(spawner)
#     spawner.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
