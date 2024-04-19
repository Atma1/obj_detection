import rclpy
import cv_bridge
from rclpy.node import Node
from random import randrange
from sensor_msgs.msg import Image
from obj_detection_interfaces.msg import Detection
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class DetectionPublisher(Node):

    def __init__(self):
        super().__init__("detection_publisher")
        self.publisher_ = self.create_publisher(Detection, "detection_node/detection", 10)
        timer = 5
        self.subscriber_ = self.create_subscription(Image, "detection", self.callback)
        self.timer = self.create_timer(timer, self.callback)
        self.bridge = cv_bridge.CvBridge()

    def callback(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        msg = self.infer(cv_image)
        self.publisher_.publish(msg=msg)
        self.get_logger().info(f"Publish class: {msg.object_class}")

    def infer(self, image):
        msg = Detection(image)
        msg.bottom_rightmost_x = randrange(0, 99)
        msg.bottom_rightmost_y = randrange(0, 99)
        msg.top_leftmost_x = randrange(0, 99)
        msg.top_leftmost_y = randrange(0, 99)
        msg.object_class = randrange(0, 255)

        return msg

def main(args=None):
    rclpy.init(args=args)
    detection_pub = DetectionPublisher()
    rclpy.spin(detection_pub)
    detection_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
