import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math


class Lidar(Node):

    def __init__(self):
        super().__init__('lidar')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        self.timer_period = 0.1  # Update frequency in seconds
        self.laser_forward = float('inf')
        self.laser_front_left = float('inf')
        self.laser_front_right = float('inf')
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self, msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[0]  # Forward (0 degrees)
    
        # Save the minimum distance in the front-left sector (0° to 15°)
        self.laser_frontLeft = min(msg.ranges[0:15])  # Front-left (0° to 15°)

        # Save the minimum distance in the front-right sector (345° to 359°)
        self.laser_frontRight = min(msg.ranges[-15:])  # Front-right (345° to 359°)

        # Log the distances for debugging
        self.get_logger().info(
            'Forward: {:.2f} m, Front-Left: {:.2f} m, Front-Right: {:.2f} m'.format(
                self.laser_forward, self.laser_frontLeft, self.laser_frontRight
            )
        )   

    def motion(self):
        # Log the current distances
        self.get_logger().info('Forward: {:.2f} m, Front-Left: {:.2f} m, Front-Right: {:.2f} m'.format(
            self.laser_forward, self.laser_front_left, self.laser_front_right))

        # Thresholds
        stop_distance = 0.6  # Minimum safe distance to stop

        # Speeds
        linear_speed = 0.0

        # Check for obstacles in the forward, front-left, or front-right directions
        if (self.laser_forward > stop_distance and
                self.laser_front_left > stop_distance and
                self.laser_front_right > stop_distance):
            # Path is clear, move forward
            linear_speed = 0.2
            self.get_logger().info('Path is clear. Moving forward.')
        else:
            # Obstacle detected, stop
            linear_speed = 0.0
            self.get_logger().info('Obstacle detected. Stopping.')

        # Set the speeds
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = 0.0  # No turning needed

        # Publish the command
        self.publisher_.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
