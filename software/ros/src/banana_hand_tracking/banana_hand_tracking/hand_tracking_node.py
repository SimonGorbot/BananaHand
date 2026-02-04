"""ROS 2 node that runs MediaPipe Hands on incoming images."""

from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from banana_interfaces.msg import HandState

try:
    import mediapipe as mp
except ImportError as exc:
    raise SystemExit(
        "mediapipe is not installed. Try: pip install mediapipe"
    ) from exc


class HandTrackingNode(Node):
    """Subscribe to /camera/image_raw and run MediaPipe Hands."""

    def __init__(self) -> None:
        super().__init__("hand_tracking_node")
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("show_preview", True)
        self.declare_parameter("mirror_handedness", True)

        input_topic = str(self.get_parameter("input_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._show_preview = bool(self.get_parameter("show_preview").value)
        self._mirror_handedness = bool(
            self.get_parameter("mirror_handedness").value
        )

        self._bridge = CvBridge()
        self._subscription = self.create_subscription(
            Image, input_topic, self._on_image, 10
        )
        self._publisher = self.create_publisher(HandState, "/hand/landmarks", 10)

        self._mp_hands = mp.solutions.hands
        self._mp_draw = mp.solutions.drawing_utils
        self._hands = self._mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        if self._show_preview:
            cv2.namedWindow("hand_tracking", cv2.WINDOW_NORMAL)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self._hands.process(rgb)

        has_hand = bool(results.multi_hand_landmarks)
        landmarks_msg = HandState()
        landmarks_msg.header.stamp = self.get_clock().now().to_msg()
        landmarks_msg.header.frame_id = self._frame_id
        landmarks_msg.landmarks = [Point() for _ in range(21)]
        landmarks_msg.handedness = "none"

        if has_hand:
            for landmarks in results.multi_hand_landmarks:
                self._mp_draw.draw_landmarks(
                    frame, landmarks, self._mp_hands.HAND_CONNECTIONS
                )
                for idx, lm in enumerate(landmarks.landmark):
                    if idx >= 21:
                        break
                    landmarks_msg.landmarks[idx].x = float(lm.x)
                    landmarks_msg.landmarks[idx].y = float(lm.y)
                    landmarks_msg.landmarks[idx].z = float(lm.z)
                break

            if results.multi_handedness:
                handedness = str(
                    results.multi_handedness[0].classification[0].label
                ).lower()
                if self._mirror_handedness:
                    handedness = "left" if handedness == "right" else "right"
                landmarks_msg.handedness = handedness
        # No hand detected: show the raw frame without stale overlays.

        self._publisher.publish(landmarks_msg)

        if self._show_preview:
            cv2.imshow("hand_tracking", frame)
            cv2.waitKey(1)

    def destroy_node(self) -> bool:
        self._hands.close()
        if self._show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HandTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
