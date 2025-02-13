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
        self.threshold_distance = 0.2

        self.right_distance = 0
        self.left_distance = 0 
        self.front_distance = 0

        self.x_position = 0
        self.y_position = 0
        self.yaw = 0

        self.wall_found = False

        self.side_chosen = None


        # publishers
        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", QOS_PROFILE)

        # subscribers
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, QOS_PROFILE)
        self.create_subscription(Odometry, "/odom", self.odometry_callback, QOS_PROFILE)
        
        # timer
        self.create_timer(0.2, self.controller)

        self.get_logger().info("Wall Follower Initialized")
    
    def lidar_callback(self, msg: LaserScan) -> None:
        front_angle = 0
        left_angle = (msg.angle_min - front_angle) / 2
        right_angle = (msg.angle_max - front_angle) / 2

        # calculating front, right, and left scan index
        left_index = int((left_angle - msg.angle_min) / (msg.angle_increment))
        front_index = int((front_angle - msg.angle_min) / (msg.angle_increment))
        right_index = int((right_angle - msg.angle_min) / (msg.angle_increment))

        self.left_distance = min(min(msg.ranges[left_index:left_index + 10]), min(msg.ranges[left_index - 10:left_index]))
        self.right_distance = min(min(msg.ranges[right_index:right_index + 10]), min(msg.ranges[right_index - 10:right_index]))
        self.front_distance = min(min(msg.ranges[front_index:front_index + 10]), min(msg.ranges[front_index - 10:front_index]))
    
    def odometry_callback(self, msg: Odometry) -> None:
        # variables
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

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

    def controller(self) -> None:
        if self.wall_found:
            if self.front_distance < self.threshold_distance:
                if self.side_chosen == "right":
                    self.turn("slow", "left")
                if self.side_chosen == "left":
                    self.turn("slow", "right")

            else:
                if self.side_chosen == "right":
                    if self.right_distance < self.threshold_distance:
                        self.turn("fast", "left")
                    elif self.right_distance > self.threshold_distance:
                        self.turn("fast", "right")
                    else:
                        self.move_forward()

                if self.side_chosen == "left":
                    if self.left_distance < self.threshold_distance:
                        self.turn("fast", "right")
                    elif self.left_distance >self.threshold_distance:
                        self.turn("fast", "left")
                    else:
                        self.move_forward()
        else:
            if self.front_distance < self.threshold_distance:
                self.wall_found = True
                self.stop_robot()

                if self.side_chosen is None:
                    if self.right_distance < self.left_distance:
                        self.side_chosen = "right"
                    else:
                        self.side_chosen = "left"
            else:
                self.move_forward()



                

    # helper functions
    def turn(self, speed: str, turn: str) -> None:
        msg = Twist()

        velocity = 0.05 if speed == "fast" else 0.02
        direction = 0.2 if turn == "left" else -0.2

        msg.linear.x = velocity
        msg.angular.z = direction

        self.velocity_pub.publish(msg)
        self.get_logger().info(f"Robot is moving to the {turn} at a slow speed: {msg.linear.x} m/s")

    def move_forward(self) -> None:
        msg = Twist()

        msg.linear.x = 0.5
        msg.angular.z = 0.0

        self.velocity_pub.publish(msg)
        self.get_logger().info(f"Robot is moving forward at a speed of {msg.linear.x} m/s")

    def stop_robot(self) -> None:
        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.velocity_pub.publish(msg)
        self.get_logger().info("Robot has stopped")

            
def main(args=None) -> None:
    rclpy.init(args=args)

    wall_follower_node = WallFollowerNode()

    rclpy.spin(wall_follower_node)
    wall_follower_node.destroy_node()
    rclpy.shutdown()