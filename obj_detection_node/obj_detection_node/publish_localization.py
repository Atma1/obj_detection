import rclpy
from rclpy.node import Node
from obj_detection_interfaces.msg import DetectionArray, Localization, LocalizationArray
from geometry_msgs.msg import Point
from util.util import get_degree, get_distance, convert_to_cm


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.declare_parameter("img_height", value=640)
        self.declare_parameter("img_width", value=640)
        self.image_height = self.get_parameter("img_height").get_parameter_value().integer_value
        self.image_width = self.get_parameter("img_width").get_parameter_value().integer_value
        self.img_size = (self.image_width, self.image_height)
        self.publisher_ = self.create_publisher(Point, "detection_node/localization", 10)
        self.subscriber_ = self.create_subscription(DetectionArray, "detection_node/detection", self.callback, 10)
        self.get_logger().info('Initializing localization publisher')

    def callback(self, detectionArray: DetectionArray):
        if detectionArray is None:
            self.get_logger().info("No detection found.")

        localization_array = LocalizationArray()

        for detection in detectionArray:
            localization = Localization()
            localization_array.localizations = []
            x1, y1, x2, y2 = detection
            bbox_xy = [x1, y1, x2, y2]
            class_id = detection.hypothesis.class_id

            angle = get_degree(self.img_size, bbox_xy)
            pixel_distance = get_distance(self.img_size, bbox_xy=bbox_xy)
            real_distance = convert_to_cm(pixel=pixel_distance)

            localization.distance = real_distance
            localization.angle = angle
            localization.class_id = class_id

            localization_array.localizations.append(localization)

        self.publisher_.publish(msg=localization_array)
        self.get_logger().info("Published ball localization")


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
