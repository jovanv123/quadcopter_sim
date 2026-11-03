import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
from geometry_msgs.msg import Point
from drone_interfaces.msg import Detection

class PerceptionNode (Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        seg_sub = message_filters.Subscriber(self, Image, '/camera/segmentation/labels_map', qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_profile_sensor_data)
        
        self.sync = message_filters.ApproximateTimeSynchronizer([seg_sub, depth_sub], 10, slop = 0.05)
        self.sync.registerCallback(self.on_image)
        
        self.pub = self.create_publisher(Image, '/detections/image', 10)
        self.closest_detection_pub = self.create_publisher(Detection, '/detections/closest', 10)

        
    def on_image(self, seg_msg: Image, depth_msg: Image):

        seg_frame = self.bridge.imgmsg_to_cv2(seg_msg, 'passthrough')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        labels = seg_frame
        if labels.ndim == 3:
            labels = seg_frame[:,:,0]

        detections = []

        for label in np.unique(labels):
            if label == 0:
                continue
            if label in [1, 2]:
                mask_label = (labels == label)
                M = cv2.moments(mask_label.astype('uint8'))
                if M['m00'] != 0:
                    mask_centroids = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
                z = float(np.nanmedian(depth_frame[mask_label]))
                detections.append((int(label), mask_centroids, z))

        vis = cv2.cvtColor((labels * 100).astype('uint8'), cv2.COLOR_GRAY2BGR)

        for label, (u, v), _ in detections:
            color = (0, 255, 0) if label == 1 else (0, 0, 255)
            cv2.circle(vis, (u, v), 5, color, -1)

        if detections:
            self.get_logger().info(
                ' | '.join(f'L{label} ({u},{v}) z={z:.2f} m' for label, (u, v), z in detections),
                throttle_duration_sec=0.5)

        detections_sorted = sorted(detections, key=lambda x: x[2])

        if detections_sorted:
            label, (u, v), z = detections_sorted[0]
            msg = Detection()
            msg.header = depth_msg.header          # was seg_msg.header → caused the TF frame error
            msg.label = label
            msg.position = Point(x=float(u), y=float(v), z=float(z))   # still (u px, v px, z m)
            self.closest_detection_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
