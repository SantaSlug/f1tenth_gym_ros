import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanAngleReader(Node):
    def __init__(self):
        super().__init__('scan_angle_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Angles we care about (in radians)
        target_angles = {
            '0°': 0.0,
            '-90°': math.radians(-90)
        }

        for label, angle in target_angles.items():
            index = int((angle - angle_min) / angle_increment)

            # Check index bounds
            if 0 <= index < len(msg.ranges):
                distance = msg.ranges[index]
                self.get_logger().info(f"Distance at {label}: {distance:.2f} m")
            else:
                self.get_logger().warn(f"{label} is out of scan range.")

def main(args=None):
    rclpy.init(args=args)
    node = ScanAngleReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
