#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import tf2_ros
from tf2_ros import TransformException

def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y, 0],
        [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x, 0],
        [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y, 0],
        [                0,                 0,                 0, 1]
    ])

class CeilingMeasurer(Node):
    def __init__(self):
        super().__init__('ceiling_measurer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # We are using raw camera sensor data, thus freeing ourselves from the Rtabmap filter!
        # Callback group to allow TF to process in background
        self.cb_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed2/zed2_sensor/points',
            self.pc_callback,
            1,
            callback_group=self.cb_group)
            
        self.global_ceiling_points = []
        self.get_logger().info('=============================================')
        self.get_logger().info('☁️ 3D CEILING MEASUREMENT TOOL STARTED! (Cumulative measurement from camera sensor)')
        self.get_logger().info('As the robot moves forward, every piece of the ceiling seen by the camera will be added to memory and the total tunnel length will be calculated.')
        self.get_logger().info('=============================================')

    def pc_callback(self, msg):
        try:
            # CRITICAL FIX: Instead of getting the LATEST (0) time TF, we must get the position 
            # at the exact millisecond the camera captured that frame (msg.header.stamp)!
            # Otherwise, as the robot moves, second-level delays will "smear" the map forward
            # and extend a 10-meter tunnel to 13-16 meters!
            t = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.5)
            )
        except TransformException as ex:
            if not hasattr(self, 'tf_warn_cnt'):
                self.tf_warn_cnt = 0
            self.tf_warn_cnt += 1
            if self.tf_warn_cnt % 50 == 0:
                self.get_logger().info(f"TF ERROR (Missing connection or connection delay): {ex}")
            return

        try:
            points_list = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([p[0], p[1], p[2]])
                
            if not points_list:
                return
        except Exception as e:
            if not hasattr(self, 'pc_warn_cnt'): self.pc_warn_cnt = 0
            self.pc_warn_cnt += 1
            if self.pc_warn_cnt % 10 == 0:
                self.get_logger().info(f"PC READ ERROR: {e}")
            return

        pts = np.array(points_list, dtype=np.float32)
        
        # LASER SCANNER LOGIC: We ensure the camera only focuses directly in front of it 
        # (robot width). However, we set its forward vision range (Z)
        # to 3.2 meters. When we set it to 6 meters, it saw high walls far ahead of the 
        # tunnel and included them; 2 meters was too short to see just above the robot. 
        # The golden mean of the two: 3.2 Meters.
        # LASER SCANNER LOGIC: Camera X (right/left), Y (down/up), Z (forward)
        # abs(X) < 0.4: Only directly in front of the robot
        # Z < 4.5: SECRET MAGICAL GOLDEN RATIO (4.5 Meters) so it doesn't mistake mountains outside for the ceiling.
        # Y < -0.2: The camera's Y-axis is downward. Minus Y is up! Only take points (the ceiling) that are 20cm and HIGHER than the camera!
        mask = (np.abs(pts[:, 0]) < 0.4) & (pts[:, 2] < 4.5) & (pts[:, 1] < -0.2)
        pts_filtered = pts[mask]

        if not hasattr(self, 'mask_print_cnt'): self.mask_print_cnt = 0
        self.mask_print_cnt += 1
        
        if len(pts_filtered) > 0:
            if self.mask_print_cnt % 10 == 0:
                self.get_logger().info(f"DEBUG: Number of points passing the ceiling mask: {len(pts_filtered)}")
        else:
            if self.mask_print_cnt % 20 == 0:
                self.get_logger().info("DEBUG: No points passing the mask (Camera doesn't see a ceiling)")
            return
            
        pts = pts_filtered
        
        # Points coming from the camera are in the camera reference frame (Camera angle, etc.).
        # We convert them to the Odom (world) frame.
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        
        mat = quaternion_to_matrix(q)
        mat[0:3, 3] = trans
        
        pts_h = np.hstack((pts, np.ones((pts.shape[0], 1))))
        pts_odom = pts_h.dot(mat.T)[:, 0:3]
        
        # NO NEED to look at the absolute odom height anymore, it is guaranteed by the masking (Y < -0.2).
        ceiling_pts = pts_odom
        
        if len(ceiling_pts) == 0:
            return

        # To prevent filling up RAM too much, let's sample only 25% of it and add it to the list
        np.random.shuffle(ceiling_pts)
        sample_size = max(1, int(len(ceiling_pts) * 0.25))
        sampled_ceiling = ceiling_pts[:sample_size]

        # Add to the global list
        self.global_ceiling_points.extend(sampled_ceiling.tolist())
        
        global_pts_arr = np.array(self.global_ceiling_points)
        
        if len(global_pts_arr) < 50:
            return

        # Only X and Y projection
        pts_2d = np.float32(global_pts_arr[:, 0:2])
        
        # To prevent Smeared (lingering camera error points at the edges) noise, 
        # instead of drawing an OpenCV Box, we will take the Principal Component (PCA) 
        # projection of the data and measure the 96% body of the distribution.
        cov = np.cov(pts_2d, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        main_axis = eigenvectors[:, np.argmax(eigenvalues)]
        
        projections = pts_2d.dot(main_axis)
        
        # Cut out and discard potential camera miscalculations (and fake hills outside the tunnel):
        length_meters = np.percentile(projections, 98) - np.percentile(projections, 2)
        
        self.get_logger().info(f"--- 3D PRECISE CEILING LENGTH: {length_meters:.2f} METERS | (Processed Points: {len(global_pts_arr)})")

def main(args=None):
    rclpy.init(args=args)
    node = CeilingMeasurer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()