#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class DebugPC(Node):
    def __init__(self):
        super().__init__('debug_pc')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed2/zed2_sensor/points',
            self.pc_callback,
            1)
        self.count = 0
        self.get_logger().info('Debugging raw pointcloud values...')

    def pc_callback(self, msg):
        self.count += 1
        if self.count % 10 != 0:
            return  # only print every 1 second
            
        try:
            points_list = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([p[0], p[1], p[2]])
            
            if not points_list:
                self.get_logger().info("NO POINTS!")
                return
                
            pts = np.array(points_list)
            self.get_logger().info(f"Points shape: {pts.shape}")
            self.get_logger().info(f"Raw X (min/max): {np.min(pts[:,0]):.2f} / {np.max(pts[:,0]):.2f}")
            self.get_logger().info(f"Raw Y (min/max): {np.min(pts[:,1]):.2f} / {np.max(pts[:,1]):.2f}")
            self.get_logger().info(f"Raw Z (depth) (min/max): {np.min(pts[:,2]):.2f} / {np.max(pts[:,2]):.2f}")
            
        except Exception as e:
            self.get_logger().info(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DebugPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
