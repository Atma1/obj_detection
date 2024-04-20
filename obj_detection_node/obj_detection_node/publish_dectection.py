import rclpy
import cv_bridge
from rclpy.node import Node
from ultralytics import YOLO
from random import randrange
from sensor_msgs.msg import Image
from obj_detection_interfaces.msg import Detection
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class DetectionPublisher(Node):

    def __init__(self):
        super().__init__("detection_node")
        self.publisher_ = self.create_publisher(Detection, "detection_node/detection", 10)
        self.subscriber_ = self.create_subscription(Image, "detection_node/image", self.callback)
        self.bridge = cv_bridge.CvBridge()
        self.model = YOLO("model_weight_path")

    def callback(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        msg = self.infer(cv_image)
        self.publisher_.publish(msg=msg)
        self.get_logger().info(f"Publish class: {msg.object_class}")

    def infer(self, image):
        result = self.model.predict(image)

        return result

def main(args=None):
    rclpy.init(args=args)
    detection_pub = DetectionPublisher()
    rclpy.spin(detection_pub)
    detection_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
