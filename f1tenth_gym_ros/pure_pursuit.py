#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped # type: ignore

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Parameters
        self.speed = 1.0  
        self.wheelbase = 0.33  
        self.prev_steering = 0.0

        # Load waypoints from raceline CSV
        waypoints_path = '/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_raceline.csv'
        self.waypoints = self.load_waypoints(waypoints_path)

        # ROS publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # ROS subscriber
        self.pose_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

    def load_waypoints(self, path):
        data = np.genfromtxt(path, delimiter=';', skip_header=0)
        return data

    def odom_callback(self, odom_msg):
        # Extract pose and orientation
        car_x = odom_msg.pose.pose.position.x
        car_y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)

        # Adaptive lookahead based on speed (temporary default before target)
        lookahead_distance = max(0.5, min(2.5, self.speed * 0.8))

        # Find forward lookahead point(scaling it dynamically)
        lookahead_point = self.find_lookahead_point(car_x, car_y, yaw, lookahead_distance)
        if lookahead_point is None:
            self.get_logger().warn("No valid lookahead point found")
            return

        self.get_logger().info(f"Lookahead point: {lookahead_point}, car: ({car_x:.2f}, {car_y:.2f})")

        # Find closest index to lookahead point (for kappa(curvature))
        dists = np.linalg.norm(self.waypoints[:, :2] - np.array(lookahead_point), axis=1)
        closest_idx = np.argmin(dists)

        # Use raceline curvature to control speed
        try:
            raceline_curvature = float(abs(self.waypoints[closest_idx][4]))
            if np.isnan(raceline_curvature):
                raceline_curvature = 0.0
        except (IndexError, ValueError):
            raceline_curvature = 0.0

        # Smooth speed ramp-up
        target_speed = max(0.5, min(2.5, 5.0 / (1.0 + 20.0 * raceline_curvature)))
        self.speed = 0.9 * self.speed + 0.1 * target_speed

        # Update lookahead distance after adjusting speed
        lookahead_distance = max(0.5, min(2.5, self.speed * 0.8))

        # Transform lookahead point to vehicle frame
        dx = lookahead_point[0] - car_x
        dy = lookahead_point[1] - car_y
        local_x = np.cos(-yaw) * dx - np.sin(-yaw) * dy
        local_y = np.sin(-yaw) * dx + np.cos(-yaw) * dy

        # Pure Pursuit steering angle calculation
        ld = np.hypot(local_x, local_y)
        if ld == 0:
            return
        curvature = 2 * local_y / (ld ** 2)
        curvature = np.clip(curvature, -2.5, 2.5)  # Clamp curvature
        steering_angle = np.arctan(self.wheelbase * curvature)

        # Apply low-pass filter to smooth steering
        steering_angle = 0.7 * self.prev_steering + 0.3 * steering_angle
        self.prev_steering = steering_angle

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(np.clip(steering_angle, -0.4189, 0.4189))  # ~24 deg max
        drive_msg.drive.speed = self.speed
        self.drive_pub.publish(drive_msg)

        self.get_logger().info(
            f"Publishing: speed={drive_msg.drive.speed:.2f}, steering={drive_msg.drive.steering_angle:.2f}, "
            f"ld={lookahead_distance:.2f}, Closest waypoint index: {closest_idx}, kappa={raceline_curvature:.5f}"
        )

    def find_lookahead_point(self, x, y, yaw, lookahead_distance):
        best_point = None
        best_score = float('inf')

        for point in self.waypoints:
            wx, wy = point[0], point[1]
            dx = wx - x
            dy = wy - y
            dist = np.hypot(dx, dy)

            if dist < lookahead_distance:
                continue

            heading_to_point = np.arctan2(dy, dx)
            angle_diff = np.arctan2(np.sin(heading_to_point - yaw), np.cos(heading_to_point - yaw))

            if abs(angle_diff) < np.pi / 2:
                score = dist * (1 + abs(angle_diff))  # penalize large angles
                if score < best_score:
                    best_point = [wx, wy]
                    best_score = score

        return best_point

    # Convert orientation from /odom to heading angle
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
