import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped # type: ignore
import math


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.0
        self.ttc_threshold = 1.0  
        self.min_projection = 0.001  # To avoid division by zero

        # Publisher to control the car
        self.publisher_vel = self.create_publisher(AckermannDriveStamped,'/drive',10)

        # Subscribers to scan and odometry
        self.sub_odometry = self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)
        self.sub_laser_scan = self.create_subscription(LaserScan,'/scan', self.scan_callback,10)

       
    def odom_callback(self, odom_msg):
        # Store current speed from odometry
        self.speed = odom_msg.twist.twist.linear.x
     
    def scan_callback(self, scan_msg):
        for i,r in enumerate(scan_msg.ranges):

            if math.isinf(r) or math.isnan(r) or r < scan_msg.range_min or r > scan_msg.range_max:
                continue
                
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            projection = self.speed * math.cos(angle)
            if projection <= 0:
                continue #because theta is greater than 90 or less than -90, so no ibstacle on front or side

            ttc = r / projection

            if ttc < self.ttc_threshold:
                self.get_logger().warn(F'WARN: about to collide in {ttc}sec Vel:= {self.speed}m/s')
                self.emergency_break()
                break
    
    def emergency_break(self):
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        self.get_logger().info(f'Vel:= {self.speed}m/s')
        self.publisher_vel.publish(brake_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




