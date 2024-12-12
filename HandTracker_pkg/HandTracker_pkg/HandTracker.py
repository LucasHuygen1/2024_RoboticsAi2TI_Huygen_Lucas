import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp


class HandTrackerNode(Node):
    def __init__(self):
        super().__init__('hand_tracker_node')
        
        # Publisher to send hand gestures
        # voor movement
        self.publisher_ = self.create_publisher(String, '/hand_gesture', 10)
        
        # Initialize Mediapipe Hand Detector
        self.detector = handDetector()

        # Open webcam
        self.cap = cv2.VideoCapture(0) 

    def run(self):
        while True:
            success, img = self.cap.read()
            if not success:
                self.get_logger().error("Failed to read from webcam")
                break

            # hands detecterten.
            img = self.detector.findHands(img)
            lmList = self.detector.findPosition(img)

            # kijk welke gesture uitgevoerd wordt
            gesture = None
            if self.detector.fingersStraight(lmList):
                gesture = "MOVE FORWARD"
            elif self.detector.handClosed(lmList):
                gesture = "STOP"
            elif self.detector.indexFingerUp(lmList):
                gesture = "ACCELERATE"
            elif self.detector.twoFingersUp(lmList):
                gesture = "SLOW DOWN"
            elif self.detector.thumbUp(lmList):
                gesture = "SPIN"

            if gesture:
                msg = String()
                msg.data = gesture
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published gesture: {gesture}")

            # image
            cv2.imshow("Hand Tracking", img)

            #q voor stoppen
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


class handDetector():
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)

        return img

    def findPosition(self, img, handno=0, draw=True):
        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handno]

            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 255), cv2.FILLED)

        return lmList

    def fingersStraight(self, lmList):
        #kijk of vingers recht zijn
        if len(lmList) == 0:
            return False

        for finger_id in [8, 12, 16, 20]:   
            tip = lmList[finger_id]
            pip = lmList[finger_id - 2]  
            if tip[2] > pip[2]:  
                return False
        return True

    def handClosed(self, lmList):
        # kijken of alle vingers toe zijn
        if len(lmList) == 0:
            return False

        for finger_id in [8, 12, 16, 20]:
            tip = lmList[finger_id]
            pip = lmList[finger_id - 2]
            if tip[2] < pip[2]:  
                return False
        return True

    def indexFingerUp(self, lmList):
        # kijk of alleen index omhoog is
        if len(lmList) == 0:
            return False
        return lmList[8][2] < lmList[6][2] and all(lmList[f][2] > lmList[f - 2][2] for f in [12, 16, 20])

    def twoFingersUp(self, lmList):
        # kijk of index en middenfinger omhoog is
        if len(lmList) == 0:
            return False
        return (lmList[8][2] < lmList[6][2] and lmList[12][2] < lmList[10][2]
                and lmList[16][2] > lmList[14][2] and lmList[20][2] > lmList[18][2])

    def thumbUp(self, lmList):
        # kijk of duim omhoog is 
        if len(lmList) == 0:
            return False
        return lmList[4][1] < lmList[3][1] and all(lmList[f][2] > lmList[f - 2][2] for f in [8, 12, 16, 20])


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackerNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
