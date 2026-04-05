import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration 

class Detection3DToMarkerNode(Node):
    def __init__(self):
        super().__init__('detection3d_to_marker_node')
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/detect_bbox3d',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(MarkerArray, '/detection_markers', 10)
        self.marker_lifetime_sec = 0.5  # Auto-erase time (prevents old markers from staying in RViz)

    def listener_callback(self, msg):
        marker_array = MarkerArray()

        for i, detection in enumerate(msg.detections):
            label_text = "unknown"
            if detection.results:
                label_text = detection.results[0].hypothesis.class_id
                score = detection.results[0].hypothesis.score
            else:
                continue

            # --- 3D BBox Marker ---
            bbox_marker = Marker()
            bbox_marker.header = msg.header
            bbox_marker.ns = 'bbox'
            bbox_marker.id = i * 2     # ID must be unique
            bbox_marker.type = Marker.CUBE
            bbox_marker.action = Marker.ADD
            bbox_marker.pose = detection.bbox.center
            bbox_marker.scale = detection.bbox.size
            if label_text == "Car":
                bbox_marker.color.a = 0.4
                bbox_marker.color.r = 1.0
                bbox_marker.color.g = 1.0
                bbox_marker.color.b = 0.0
            elif label_text == "Pedestrian":
                bbox_marker.color.a = 0.4
                bbox_marker.color.r = 1.0
                bbox_marker.color.g = 0.0
                bbox_marker.color.b = 1.0
            else:
                bbox_marker.color.a = 0.4
                bbox_marker.color.r = 0.0
                bbox_marker.color.g = 1.0
                bbox_marker.color.b = 1.0
            bbox_marker.lifetime = Duration(sec=0, nanosec=100_000_000)  # 0.1 seconds

            marker_array.markers.append(bbox_marker)

            # --- Text Marker (Label Name + Score) ---
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = 'label'
            text_marker.id = i * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = detection.bbox.center
            text_marker.pose.position.z += detection.bbox.size.z * 0.6  # Display above the box
            text_marker.scale.z = 0.8  # Text size
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"{label_text} ({score:.2f})"
            text_marker.lifetime = Duration(sec=0, nanosec=100_000_000)  # 0.1 seconds

            marker_array.markers.append(text_marker)

        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Detection3DToMarkerNode()
    rclpy.spin(node)
    rclpy.shutdown()
