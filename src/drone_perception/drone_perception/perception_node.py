import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PerceptionNode (Node);
    def __init__(self):
        super().__init__('perception node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, '/detections/image', 10)

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # PASS TO YOLO OR SOME OTHER MODEL #
        out.header = msg.header
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args)
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()
