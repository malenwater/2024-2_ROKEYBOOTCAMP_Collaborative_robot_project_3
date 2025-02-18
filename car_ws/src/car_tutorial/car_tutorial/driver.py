import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image

class Driver(Node):
    def __init__(self):
        super().__init__("drive")
        
        # Parameters
        self.wheel_separation = 0.1
        self.wheel_diameter = 0.03
        self.timer_frequently = 0.1
        
        # Init variable
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_states.position = [0.0, 0.0]
        
        self.linear = 0.0
        self.angular = 0.0
        
        self.wheel_speed = [0.0, 0.0]
        self.wheel_rotate = [0.0, 0.0]
        
        self.msg = Twist()
        self.raw_vel = Twist()
        
        self.bridge = CvBridge()
        self.image = cv2.imread("src/car_tutorial/photo/photo.png", cv2.IMREAD_COLOR)
        self.img_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        
        # Subscriber
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        
        # Publisher
        self.pub_vel_raw = self.create_publisher(Twist, "raw_vel", 10)
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)
        self.pub_img = self.create_publisher(Image, "image", 5)
        
        # Timer
        self.timer_1 = self.create_timer(self.timer_frequently, self.publish_jointstate)
        self.timer_2 = self.create_timer(self.timer_frequently, self.publish_raw_vel)
        self.timer_3 = self.create_timer(5.0, self.publish_image)
    
    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"Received cmd_vel message {msg}")
        self.msg = msg
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        
    def publish_raw_vel(self):
        self.raw_vel = self.msg
        self.pub_vel_raw.publish(self.raw_vel)
        
    def publish_jointstate(self):
        # Wheel speed
        self.wheel_speed[0] = self.linear - self.angular * self.wheel_separation / 2.0
        self.wheel_speed[1] = self.linear + self.angular * self.wheel_separation / 2.0
        
        # Wheel rotate speed
        self.wheel_rotate[0] = self.wheel_speed[0] / (self.wheel_diameter / 2.0)
        self.wheel_rotate[1] = self.wheel_speed[1] / (self.wheel_diameter / 2.0)
        
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position[0] = self.wheel_rotate[0] * self.timer_frequently
        self.joint_states.position[1] = self.wheel_rotate[1] * self.timer_frequently
        
        # Publish
        self.pub_joint_states.publish(self.joint_states)
        
    def publish_image(self):
        try:
            self.pub_img.publish(self.img_msg)
        except:
            return

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    executor = MultiThreadedExecutor()
    rclpy.spin(driver, executor=executor)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()