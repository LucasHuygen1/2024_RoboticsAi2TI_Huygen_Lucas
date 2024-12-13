import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # Subscribing to hand gestures and lidar data
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT)
        self.create_subscription(String, '/hand_gesture', self.gesture_callback, qos_profile)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        # Publisher to send commands to the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Default speeds
        self.speed = 0.2
        self.turn_speed = 0.5

        # Patrol status
        self.patrol_active = False

        # Lidar data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # Distance threshold for obstacles
        self.stop_distance = 0.6  # Minimum safe distance in meters

        # Timer for periodic motion updates
        self.timer = self.create_timer(0.1, self.update_motion)

        # Track the last sent command
        self.last_twist = Twist()

        self.get_logger().info("Movement node initialized!")

    def gesture_callback(self, msg):
        """
        Callback for hand gestures.
        """
        command = msg.data

        if command == "VOORUIT":
            self.patrol_active = True
            self.get_logger().info("Gesture: VOORUIT (Patrol started)")
        elif command == "STOP":
            self.patrol_active = False
            self.stop_robot()
            self.get_logger().info("Gesture: STOP (Patrol stopped)")

    def lidar_callback(self, msg):
        """
        Process Lidar data.
        """
        self.front_distance = min(msg.ranges[0:20] + msg.ranges[-20:])
        self.left_distance = min(msg.ranges[60:100])
        self.right_distance = min(msg.ranges[260:300])

        # Log distances for debugging
        self.get_logger().info(
            f"Lidar - Front: {self.front_distance:.2f} m, Left: {self.left_distance:.2f} m, Right: {self.right_distance:.2f} m"
        )

    def update_motion(self):
        """
        Perform patrol and avoid obstacles.
        """
        if self.patrol_active:
            twist = Twist()

            if self.front_distance < self.stop_distance:
                self.get_logger().info("Obstacle detected ahead. Turning...")
                if self.left_distance > self.right_distance:
                    twist.angular.z = self.turn_speed  # Turn left
                else:
                    twist.angular.z = -self.turn_speed  # Turn right
            else:
                self.get_logger().info("Clear path. Moving forward.")
                twist.linear.x = self.speed

            # Only publish if the command has changed
            if twist.linear.x != self.last_twist.linear.x or twist.angular.z != self.last_twist.angular.z:
                self.cmd_vel_publisher.publish(twist)
                self.last_twist = twist
        else:
            self.stop_robot()

    def stop_robot(self):
        """
        Stop the robot.
        """
        twist = Twist()
        if twist.linear.x != self.last_twist.linear.x or twist.angular.z != self.last_twist.angular.z:
            self.cmd_vel_publisher.publish(twist)
            self.last_twist = twist
        self.get_logger().info("Robot stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
