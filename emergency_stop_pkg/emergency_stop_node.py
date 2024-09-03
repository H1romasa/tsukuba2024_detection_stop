import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from .object_detection import detect_objects
from .config import STOP_DISTANCE, CMD_VEL_TOPIC
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import logging

# カメラトピックを直接ここで定義
CAMERA_TOPIC = '/camera/camera/color/image_raw'

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.get_logger().set_level(logging.DEBUG)
        self.bridge = CvBridge()

        # QoSプロファイルの定義
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_profile)
        self.camera_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.camera_callback,
            qos_profile
        )
        self.get_logger().info('Emergency Stop Node has been started')

    def camera_callback(self, msg):
        self.get_logger().debug(f"Received image with timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        objects = detect_objects(cv_image)
        
        self.get_logger().debug(f"Detected objects: {objects}")
        
        closest_distance = float('inf')
        for obj in objects:
            if obj['distance'] < closest_distance:
                closest_distance = obj['distance']
        
        self.get_logger().debug(f"Closest distance: {closest_distance}")
        
        if closest_distance <= STOP_DISTANCE:
            self.emergency_stop(closest_distance)
        else:
            self.resume_movement()

    def emergency_stop(self, closest_distance):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().warn(f'Emergency stop triggered! Object detected at {closest_distance:.2f} meters')

    def resume_movement(self):
        # ここでは何もしない。jmoab_ros2パッケージが通常の動作を制御する
        pass

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()