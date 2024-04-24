import rclpy
import cv_bridge
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from obj_detection_interfaces.msg import Detection
from vision_msgs.msg import Detection2DArray, ObjectHypothesis

class DetectionPublisher(Node):

    def __init__(self):
        super().__init__("detection_node")
        self.model_name = "yolov8n408.pt"
        self.path = get_package_share_directory("obj_detection_node")
        self.publisher_ = self.create_publisher(Detection2DArray, "detection_node/detection", 10)
        self.subscriber_ = self.create_subscription(Image, "detection_node/image", self.callback)
        self.bridge = cv_bridge.CvBridge()
        self.model = YOLO(f"{self.path}/models/{self.model_name}")

    def callback(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        results = self.infer(cv_image)
        if results is not None:
            msg = self.create_message(results=results)
            self.publisher_.publish(msg=msg)
            self.get_logger().info(f"Published inference")

    def infer(self, image):
        result = self.model.predict(image)

        return result
    
    def create_message(self, results):
        detection_msg = Detection2DArray()
        bounding_box = results[0].boxes.xyxy
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf

        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
            detection = Detection()
            detection.top_leftmost_x = float(bbox[0])
            detection.top_leftmost_y = float(bbox[1])
            detection.bottom_rightmost_x = float(bbox[2])
            detection.bottom_rightmost_y = float(bbox[3])
            hypothesis = ObjectHypothesis()
            hypothesis.class_id = results[0].names.get(int(cls))
            hypothesis.score = float(conf)
            detection.hypothesis = hypothesis
            detection_msg.detections.append(detection)

        return detection_msg


def main(args=None):
    rclpy.init(args=args)
    detection_pub = DetectionPublisher()
    rclpy.spin(detection_pub)
    detection_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
