import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detector')
        self.bridge = CvBridge()

        # Subscriptions
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)

        # Publishers
        self.center_pub = self.create_publisher(PointStamped, '/shape_center', 10)
        self.edge_pub = self.create_publisher(Image, '/edges', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/shape_markers', 10)

        self.last_depth_frame = None

    def depth_callback(self, msg):
        self.last_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.last_depth_frame is None:
            return

        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        depth_image = self.last_depth_frame

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        edges = cv2.Canny(blurred, 50, 150)

        # Publish edge image
        edge_msg = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
        edge_msg.header.stamp = msg.header.stamp
        edge_msg.header.frame_id = "camera_link"
        self.edge_pub.publish(edge_msg)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        markers = MarkerArray()

        for i, contour in enumerate(contours):
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            center = self.get_center_of_contour(contour)
            if center is None:
                continue

            x, y = center
            if y >= depth_image.shape[0] or x >= depth_image.shape[1]:
                continue

            depth_value = depth_image[y, x]
            distance = depth_value / 1000.0  # mm to meters

            if self.is_rectangle(approx) or self.is_circle(contour, approx):
                # Publish PointStamped
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = "camera_link"
                point_msg.point.x = float(x)
                point_msg.point.y = float(y)
                point_msg.point.z = float(distance)
                self.center_pub.publish(point_msg)

                # Publish marker
                marker = Marker()
                marker.header.frame_id = "camera_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(x) / 1000.0
                marker.pose.position.y = float(y) / 1000.0
                marker.pose.position.z = float(depth_value) / 1000.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def get_center_of_contour(self, contour):
        M = cv2.moments(contour)
        if M["m00"] != 0:
            return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        return None

    def is_rectangle(self, approx):
        return len(approx) == 4 and cv2.isContourConvex(approx)

    def is_circle(self, contour, approx):
        if len(approx) >= 7:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                return False
            circularity = 4 * np.pi * area / (perimeter ** 2)
            return 0.7 < circularity < 1.2
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()