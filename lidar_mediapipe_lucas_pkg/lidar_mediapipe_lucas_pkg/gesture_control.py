import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import mediapipe as mp
import cv2


class GestureControl(Node):
    def __init__(self):
        super().__init__('gesture_control')
        self.publisher = self.create_publisher(String, '/hand_gesture', 10)
        self.timer = self.create_timer(0.1, self.detect_gesture)

        # Mediapipe initialiseren
        self.hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
        self.drawing_utils = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)  # Webcam openen

    def detect_gesture(self):
        # Webcam frame lezen
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Kan de webcam niet openen")
            return

        # Frame verwerken met Mediapipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Controleer of hand open (5 vingers) of gesloten (vuist) is
                fingers_up = self.count_fingers(hand_landmarks.landmark)

                if fingers_up == 5:
                    self.publish_command("VOORUIT")  # Hand open: vooruit
                elif fingers_up == 0:
                    self.publish_command("STOP")  # Vuist: stop

                # Tekenen voor debugging (optioneel)
                self.drawing_utils.draw_landmarks(frame, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)

        # Toon webcam feed
        cv2.imshow('Hand Gestures', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            self.destroy_node()

    def count_fingers(self, landmarks):
        """
        Vingers tellen met Mediapipe landmarks.
        """
        tip_ids = [4, 8, 12, 16, 20]
        fingers = []

        # Duim
        if landmarks[tip_ids[0]].x < landmarks[tip_ids[0] - 1].x:
            fingers.append(1)
        else:
            fingers.append(0)

        # Andere vingers
        for i in range(1, 5):
            if landmarks[tip_ids[i]].y < landmarks[tip_ids[i] - 2].y:
                fingers.append(1)
            else:
                fingers.append(0)

        return sum(fingers)

    def publish_command(self, command):
        """
        Publiceer 'VOORUIT' of 'STOP'.
        """
        self.publisher.publish(String(data=command))
        self.get_logger().info(f"Command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = GestureControl()
    rclpy.spin(node)
    rclpy.shutdown()
