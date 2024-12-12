import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # subscriber om te zien want de gesture is
        self.subscription = self.create_subscription(
            String,
            '/hand_gesture',  # Topic van HandTracker
            self.listener_callback,
            10
        )

        # commands naar robot sturen
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # normale speeld
        self.speed = 0.5  # vooruit speed
        self.spin_speed = 1.0  # spin speed
        self.get_logger().info("Movement Node has started!")

    def listener_callback(self, msg):
        """
        Callback function triggered when a message is received on /hand_gesture.
        """
        command = msg.data  
        twist = Twist()  

        if command == "MOVE FORWARD":
            twist.linear.x = self.speed  # vooruit
            twist.angular.z = 0.0  # 
            self.get_logger().info("Command received: MOVE FORWARD")
        elif command == "STOP":
            twist.linear.x = 0.0  # Stop
            twist.angular.z = 0.0  # 
            self.get_logger().info("Command received: STOP")
        elif command == "ACCELERATE":
            self.speed = min(self.speed + 0.1, 1.0)  # omhoog speed, max 1.0
            twist.linear.x = self.speed
            self.get_logger().info(f"Command received: ACCELERATE, New Speed: {self.speed}")
        elif command == "SLOW DOWN":
            self.speed = max(self.speed - 0.1, 0.1)  # omlaag speed, min 0.1
            twist.linear.x = self.speed
            self.get_logger().info(f"Command received: SLOW DOWN, New Speed: {self.speed}")
        elif command == "SPIN":
            twist.linear.x = 0.0  #niks
            twist.angular.z = self.spin_speed  # Spin 
            self.get_logger().info("Command received: SPIN")
        else:
            self.get_logger().warn(f"Unknown command: {command}")
            return

        # naar robot publishen
        self.publisher_.publish(twist)
        self.get_logger().info(f"Published velocity: linear={twist.linear.x}, angular={twist.angular.z}")


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


if __name__ == "__main__":
    main()
