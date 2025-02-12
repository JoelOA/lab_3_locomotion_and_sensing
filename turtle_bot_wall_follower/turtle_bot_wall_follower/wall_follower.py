from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf_transformations import euler_from_quaternion

QOS_PROFILE = QoSProfile(depth=1)

class WallFollowerNode(Node):


    def __init__(self) -> None:
        """
        Initialize the WallFollowerNode Class
        """

        super().__init__("wall_follower_node")

        # variable
        self.threshold_distance = 0.5

        self.right_distances = 0
        self.left_distances = 0 
        self.front_distances = 0

        self.x_position = 0
        self.y_position = 0
        self.yaw = 0

        # publishers
        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", QOS_PROFILE)

        # subscribers
        self.create_subscription(LaserScan, "/scan", self.lidar_callback)
        self.create_subscription(Odometry, "/odom", self.odometry_callback)
        
        # timer
        self.create_timer(0.5, self.controller)

        self.get_logger().info("Wall Follower Initialized")
    
    def lidar_callback(self, msg: LaserScan) -> None:
        front_angle = 0
        left_angle = (msg.angle_min - front_angle) / 2
        right_angle = (msg.angle_max - front_angle) / 2

        # calculating front, right, and left scan index
        left_index = int((left_angle - msg.angle_min) / (msg.angle_increment))
        front_index = int((front_angle - msg.angle_min) / (msg.angle_increment))
        right_index = int((right_angle - msg.angle_min) / (msg.angle_increment))

        self.left_distances = min(min(msg.ranges[left_index:left_index + 10]), min(msg.ranges[left_index - 10:left_index]))
        self.right_distances = min(min(msg.ranges[right_index:right_index + 10]), min(msg.ranges[right_index - 10:right_index]))
        self.front_distances = min(min(msg.ranges[front_index:front_index + 10]), min(msg.ranges[front_index - 10:front_index]))
    
    def odometry_callback(self, msg: Odometry) -> None:
        # variables
        x = msg.pose.pose.position.x
        y = msg.pose.pose.posiition.y

        # getting tuple of quaternion x,y,z,w
        quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,

            
        )

        # converting to euler
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.yaw = yaw

        # calculate distance

        #self.x_position = msg.pose.pose.position.x
        #self.y_position = msg.pose.pose.position.y

        return 



    def controller(self) -> None:
        pass


def main(args=None) -> None:
    rclpy.init(args=args)

    movement_node = WallFollowerNode()

    rclpy.spin(movement_node)
    movement_node.destroy_node()
    rclpy.shutdown()