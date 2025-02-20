#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from gazebo_msgs.srv import SpawnEntity  # Gazebo SpawnEntity 서비스 사용
from geometry_msgs.msg import Pose

class RandomBoxSpawner(Node):
    def __init__(self):
        super().__init__('random_box_spawner')

        # SpawnEntity 서비스 클라이언트 생성
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🔄 Waiting for spawn_entity service...')

        self.get_logger().info('✅ SpawnEntity service available!')

    def spawn_box(self, box_name, x, y, z):
        """Gazebo에 네모난 박스 스폰 (노란색)"""
        request = SpawnEntity.Request()
        request.name = box_name  # 박스 이름

        # Gazebo SDF 모델 정의 (노란색 박스)
        request.xml = f"""
        <sdf version="1.6">
            <model name="{box_name}">
                <static>true</static>
                <link name="box_link">
                    <visual>
                        <geometry>
                            <box>
                                <size>0.5 0.5 0.5</size>  <!-- 네모난 박스 크기 -->
                            </box>
                        </geometry>
                        <material>
                            <ambient>1 1 0 1</ambient>  <!-- 노란색 -->
                            <diffuse>1 1 0 1</diffuse>  <!-- 노란색 -->
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        """

        # 박스 위치 설정
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        # Gazebo에 박스 스폰 요청
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"✅ Successfully spawned {box_name} at ({x}, {y}, {z})")
        else:
            self.get_logger().error("❌ Failed to spawn box")

def main(args=None):
    rclpy.init(args=args)
    spawner = RandomBoxSpawner()

    # ✅ 노란색 네모난 박스 하나 생성 (좌표: x=1.0, y=2.0, z=0.25)
    spawner.spawn_box("yellow_box", 2.0, 2.0, 10.0)

    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
