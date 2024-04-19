import rclpy
from rclpy.node import Node
from random import randrange
from obj_detection_interfaces.msg import Detection

class DetectionPublisher(Node):

    def __init__(self):
        super().__init__("detection_publisher")
        self.publisher_ = self.create_publisher(Detection, "detection_publisher/detection", 10)
        timer = 5
        self.timer = self.create_timer(timer, self.callback)

    def callback(self):
        msg = self.infer()
        self.publisher_.publish(msg=msg)
        self.get_logger().info(f"Publish class: {msg.object_class}")

    def infer(self):
        msg = Detection()
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
