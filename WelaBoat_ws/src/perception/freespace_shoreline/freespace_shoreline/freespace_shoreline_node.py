#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2

class ShorelineExtractorNode(Node):
    def __init__(self):
        super().__init__('freespace_shoreline')

        # Parameters
        self.declare_parameter('input_topic', '/fused/freespace')
        self.declare_parameter('output_marker_topic', '/freespace/shoreline/marker')
        self.declare_parameter('grid_resolution', 0.1)      # meters per pixel
        self.declare_parameter('grid_size', 50.0)           # meters (side length, centered at origin)
        self.declare_parameter('min_contour_points', 10)    # min points to consider a contour valid
        self.declare_parameter('z_min', -0.5)
        self.declare_parameter('z_max', 0.5)

        input_topic = self.get_parameter('input_topic').value
        output_marker_topic = self.get_parameter('output_marker_topic').value

        self.grid_res = self.get_parameter('grid_resolution').value
        self.grid_size = self.get_parameter('grid_size').value
        self.min_contour_pts = self.get_parameter('min_contour_points').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        self.half_size = self.grid_size / 2.0
        self.grid_shape = int(self.grid_size / self.grid_res)

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )
        self.marker_pub = self.create_publisher(Marker, output_marker_topic, 10)

        self.get_logger().info(f"Subscribed to {input_topic}")
        self.get_logger().info(f"Publishing shoreline marker to {output_marker_topic}")

    def pointcloud_callback(self, msg: PointCloud2):
        # Convert PointCloud2 to Nx3 numpy array (x, y, z)
        try:
            points = []
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            points = np.array(points)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse point cloud: {e}")
            return

        if points.size == 0:
            self.get_logger().debug("Empty point cloud received.")
            return

        # Optional: filter by height (Z)
        mask_z = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        points = points[mask_z]
        if points.size == 0:
            return

        # Project to BEV (X-Y plane)
        xy = points[:, :2]  # (N, 2)

        # Filter points within grid bounds
        mask_in_range = (
            (xy[:, 0] >= -self.half_size) &
            (xy[:, 0] <= self.half_size) &
            (xy[:, 1] >= -self.half_size) &
            (xy[:, 1] <= self.half_size)
        )
        xy = xy[mask_in_range]
        if xy.size == 0:
            return

        # Convert to grid indices
        indices = ((xy + self.half_size) / self.grid_res).astype(np.int32)
        # Clip to valid range (shouldn't be needed due to mask, but safe)
        indices = np.clip(indices, 0, self.grid_shape - 1)

        # Create binary occupancy grid
        grid = np.zeros((self.grid_shape, self.grid_shape), dtype=np.uint8)
        grid[indices[:, 1], indices[:, 0]] = 255  # Note: OpenCV uses (row, col) = (y, x)

        # Optional: morphological closing to fill small holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour (or all above threshold)
        valid_contours = []
        for cnt in contours:
            if len(cnt) >= self.min_contour_pts:
                valid_contours.append(cnt)

        if not valid_contours:
            self.get_logger().debug("No valid contours found.")
            # Publish empty marker to clear old ones
            self.publish_empty_marker(msg.header)
            return

        # For simplicity, use the largest contour
        largest_contour = max(valid_contours, key=cv2.contourArea)

        # Convert contour (pixel indices) back to world coordinates
        # Note: contour shape is (N, 1, 2) → (N, 2)
        contour_pixels = largest_contour.reshape(-1, 2)  # (u, v) where u=x_idx, v=y_idx
        # In our grid: x = u * res - half_size, y = v * res - half_size
        world_xy = contour_pixels.astype(np.float32) * self.grid_res - self.half_size
        # Swap x and y? No: we stored [x_idx, y_idx] → but OpenCV returns (x, y) as (col, row) = (x_idx, y_idx)
        # So: x_world = x_idx * res - half, y_world = y_idx * res - half → correct.

        # Create 3D points (set z=0 for visualization)
        shoreline_points = []
        for x, y in world_xy:
            shoreline_points.append([float(x), float(y), 0.0])

        # Publish as Marker.LINE_STRIP
        marker = Marker()
        marker.header = msg.header  # same frame_id and timestamp
        marker.ns = "shoreline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # line width
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # blue
        marker.color.a = 1.0
        marker.points = [self.create_point(x, y, z) for x, y, z in shoreline_points]
        self.marker_pub.publish(marker)

    def create_point(self, x, y, z):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def publish_empty_marker(self, header):
        marker = Marker()
        marker.header = header
        marker.ns = "shoreline"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ShorelineExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()