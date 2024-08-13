import rclpy
import cv_bridge
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from obj_detection_interfaces.msg import Detection, DetectionArray
from vision_msgs.msg import ObjectHypothesis
from util.util import center_crop


class DetectionPublisher(Node):
    def __init__(self):
        super().__init__("detection_node")
        self.declare_parameter("model_name", "yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.7)
        self.model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self.conf_threshold = self.get_parameter("conf_threshold").get_parameter_value().double_value
        self.path = get_package_share_directory("obj_detection_node")
        self.publisher_ = self.create_publisher(DetectionArray,
                                                "detection_node/detection", 10)
        self.subscriber_ = self.create_subscription(Image,
                                                    "detection_node/image",
                                                    self.callback, 10)
        self.bridge = cv_bridge.CvBridge()
        self.model = YOLO(f"{self.path}/models/{self.model_name}")
        self.get_logger().info('Initializing detection publisher')

    def callback(self, image: Image) -> None:
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        center_cropped = center_crop(cv_image, (640, 640))
        results = self.infer(center_cropped)
        if results is not None:
            msg = self.create_message(results=results)
            self.publisher_.publish(msg=msg)
            self.get_logger().info("Published inference")
        else:
            self.get_logger().info("No detection.")

    def infer(self, image):
        result = self.model.predict(image, conf=self.conf_threshold)

        return result

    def create_message(self, results):
        detection_array = DetectionArray()
        bounding_box = results[0].boxes.xyxy
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf

        detection_array.results = []

        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
            detection = Detection()
            detection.top_leftmost_x = int(bbox[0])
            detection.top_leftmost_y = int(bbox[1])
            detection.bottom_rightmost_x = int(bbox[2])
            detection.bottom_rightmost_y = int(bbox[3])

            hypothesis = ObjectHypothesis()
            hypothesis.class_id = str(results[0].names.get(int(cls)))
            hypothesis.score = float(conf)
            detection.hypothesis = hypothesis
            detection_array.results.append(detection)

        print(detection_array)
        return detection_array


def main(args=None):
    rclpy.init(args=args)
    detection_pub = DetectionPublisher()
    rclpy.spin(detection_pub)
    detection_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
