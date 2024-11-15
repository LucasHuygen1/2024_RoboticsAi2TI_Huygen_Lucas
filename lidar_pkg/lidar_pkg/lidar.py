import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time

class Lidar(Node):

    def __init__(self):
        super().__init__('lidar')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer_period = 0.1  # Updatefrequentie in seconden
        self.laser_forward = float('inf')
        self.laser_left = float('inf')
        self.laser_right = float('inf')
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

        # variabelen voor status
        self.is_turning = False
        self.turn_start_time = None
        self.turn_duration = 2.0  # Duur van het draaien in seconden
        self.turning_direction = 0  # 1 voor links, -1 voor rechts

    def laser_callback(self, msg):
        num_readings = len(msg.ranges)
        angle_increment = (msg.angle_max - msg.angle_min) / num_readings

        def angle_to_index(angle_deg):
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            return index % num_readings

        index_front = angle_to_index(0)
        index_left = angle_to_index(90)
        index_right = angle_to_index(-90)

        # Afstand voor
        self.laser_forward = msg.ranges[index_front]

        # Linkerkant van +45° tot +135°
        left_indices = [angle_to_index(deg) for deg in range(45, 135)]
        left_ranges = [msg.ranges[i] for i in left_indices
                       if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i])]
        self.laser_left = min(left_ranges) if left_ranges else float('inf')

        # Rechterkant van -135° tot -45°
        right_indices = [angle_to_index(deg) for deg in range(-135, -45)]
        right_ranges = [msg.ranges[i] for i in right_indices
                        if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i])]
        self.laser_right = min(right_ranges) if right_ranges else float('inf')

    def motion(self):
        # Log de huidige afstanden
        self.get_logger().info('Voor: {:.2f} m, Links: {:.2f} m, Rechts: {:.2f} m'.format(
            self.laser_forward, self.laser_left, self.laser_right))

        # Drempelwaarden
        safe_distance = 0.6  # Minimale veilige afstand voor obstakels vooruit
        min_speed_distance = 0.3  # Afstand om de robot te stoppen
        side_safe_distance = 0.5  # Minimale veilige afstand aan de zijkanten
        hysteresis = 0.05  # Kleine buffer om oscillatie te voorkomen

        # Snelheden
        linear_speed = 0.0
        angular_speed = 0.0

        if self.is_turning:
            # Ga door met draaien
            elapsed_time = time.time() - self.turn_start_time
            if elapsed_time < self.turn_duration:
                self.get_logger().info('Aan het draaien... {:.2f} seconden verstreken.'.format(elapsed_time))
                linear_speed = 0.0
                angular_speed = self.turning_direction * 0.5  # Draai met matige snelheid
            else:
                # Draaien voltooid
                self.is_turning = False
                self.get_logger().info('Draaien voltooid.')
        else:
            # Controleer of er een obstakel voor is
            if self.laser_forward > safe_distance:
                # Weg is vrij, ga vooruit
                linear_speed = 0.2
                self.get_logger().info('Weg is vrij. Gaat vooruit.')

                # Zij obstakelvermijding
                side_diff = self.laser_left - self.laser_right

                if (self.laser_left < side_safe_distance + hysteresis or
                        self.laser_right < side_safe_distance + hysteresis):
                    # Obstakels gedetecteerd aan de zijkanten
                    angular_speed = 0.5 * (side_diff / (self.laser_left + self.laser_right))
                    angular_speed = max(-0.5, min(0.5, angular_speed))
                    self.get_logger().info('Koers aanpassen om zij-obstakel te vermijden.')
                else:
                    angular_speed = 0.0  # Geen zij obstakels, ga rechtdoor
            elif self.laser_forward > min_speed_distance:
                # Obstakel gedetecteerd voor, draaien
                linear_speed = 0.0
                angular_speed = 0.0
                self.is_turning = True
                self.turn_start_time = time.time()
                # Bepaal draairichting op basis van welke zijde meer ruimte heeft
                if self.laser_left > self.laser_right:
                    self.turning_direction = 1  # Draai links
                    self.get_logger().info('Obstakel voor. Start met linksom draaien.')
                else:
                    self.turning_direction = -1  # Draai rechts
                    self.get_logger().info('Obstakel voor. Start met rechtsom draaien.')
            else:
                # Obstakel dichtbij, stoppen en draaien
                linear_speed = 0.0
                angular_speed = 0.0
                self.is_turning = True
                self.turn_start_time = time.time()
                if self.laser_left > self.laser_right:
                    self.turning_direction = 1  # Draai links
                    self.get_logger().info('Obstakel erg dichtbij! Start met linksom draaien.')
                else:
                    self.turning_direction = -1  # Draai rechts
                    self.get_logger().info('Obstakel erg dichtbij! Start met rechtsom draaien.')

        # Stel de snelheden in
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed

        # Publiceer het commando
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
