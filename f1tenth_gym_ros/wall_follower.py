import rclpy
from rclpy.node import Node

import math

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped # type: ignore

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        self.publisher_drive = self.create_publisher(AckermannDriveStamped,'/drive',10)

        self.subscriber_laser_scan = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.sub_odometry = self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback,10)

        #timer for debugging
        self.create_timer(1,self.callback_timer)

        self.desired_distance = 0.8
        self.speed = 0.0
        self.index=0
        self.beam_a_angle = 0
        self.beam_b_angle = 0
        self.dist_beam_a = 0
        self.dist_beam_b = 0

        self.kp = 0.7 #7
        self.kd = 0.2 #2
        self.ki = 0.0
        
        self.integral = 0
        self.prev_error = 0

        self.err = 0
        self.steering_angle = 0
        self.D_t = 0

    def callback_timer(self):
        self.get_logger().info(f'Error: {self.err:.3f}, D_t: {self.D_t:.3f}, Steering: {self.steering_angle:.3f}')


    def odom_callback(self, odom_msg):
        # Store current speed from odometry
        self.speed = odom_msg.twist.twist.linear.x

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        ## We are receiving msg.ranges and theta, we wont need msg.ranges for anyhting
        alpha = math.atan2(self.dist_beam_a * math.cos(angle) - self.dist_beam_b, self.dist_beam_a * math.sin(angle))

        D_t = self.dist_beam_b * math.cos(alpha)

        D_t_plus_1 = D_t + (self.speed * 1) * math.sin(alpha) # self.speed * 1 is distance travelled in 1 sec

        
        return D_t

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        ## We get msg.ranges and D_t_plus_1, we wont use range_data for anything
        
        error = self.desired_distance - dist
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        p_term = self.kp * error

        self.integral += error
        self.integral = max(min(self.integral, 1.0), -1.0)  # clamp to [-1, 1] 

        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error)
        self.prev_error = error

        steering_angle = p_term + i_term + d_term

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = - steering_angle
        drive_msg.drive.speed = 1.0 #m/s
        self.publisher_drive.publish(drive_msg)

        return steering_angle

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # 1) Compute index for beam A directly
        angle_a = math.radians(40.0) # this is angle from x axis of car ie angle from where car is pointing
        index_beam_a = int((angle_a - msg.angle_min) / msg.angle_increment)
        if not 0 <= index_beam_a < len(msg.ranges):
             return  # out of bounds

        dist_a = msg.ranges[index_beam_a]
        if math.isinf(dist_a) or math.isnan(dist_a) or dist_a < msg.range_min or dist_a > msg.range_max:
             return  # invalid beam A

        self.dist_beam_a = dist_a
        self.beam_a_angle = angle_a  # or msg.angle_min + index_beam_a*msg.angle_increment

        # 2) For beam B, let's say we want 50° more
        beam_b_offset = int(math.radians(50.0) / msg.angle_increment)  # how many steps ~ 50 degrees
        index_beam_b = index_beam_a + beam_b_offset
        if not 0 <= index_beam_b < len(msg.ranges):
            return # out of bounds

        dist_b = msg.ranges[index_beam_b]
        if math.isinf(dist_b) or math.isnan(dist_b) or dist_b < msg.range_min or dist_b > msg.range_max:
            return  # invalid beam B

        self.dist_beam_b = dist_b
        self.beam_b_angle = msg.angle_min + index_beam_b*msg.angle_increment

        # 3) Now compute alpha, D_t, error, etc.
        theta = abs(self.beam_b_angle - self.beam_a_angle)
        # D_t_plus_1 = self.get_range(msg.ranges, theta)
        # self.err = self.get_error(msg.ranges, D_t_plus_1)
        self.D_t = self.get_range(msg.ranges, theta)
        self.err = self.get_error(msg.ranges, self.D_t)
        self.steering_angle = self.pid_control(self.err , self.speed)



def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()