#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MapWallMeasurer(Node):
    def __init__(self):
        super().__init__('map_wall_measurer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1)
            
        # TF (Transform) listener initialized to find the robot's current position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(' LIVE AND "FOCUS LOCKED" MEASUREMENT STARTED!')
        self.get_logger().info('Only the walls around the robot\'s tunnel will be isolated and measured...')

    def map_callback(self, msg):
        # Get the robot's current position on the map (So it only focuses on the tunnel it is currently in)
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
        except Exception as e:
            self.get_logger().info("Waiting for robot's position (TF)...")
            return

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Convert the robot's metric coordinates to map pixel coordinates
        robot_px = int((robot_x - origin_x) / resolution)
        robot_py = int((robot_y - origin_y) / resolution)

        # OccupancyGrid data
        data = np.array(msg.data, dtype=np.int8).reshape(height, width)
        
        # Convert walls to pixels
        img = np.zeros((height, width), dtype=np.uint8)
        img[data >= 50] = 255

        # Find contours (shapes/walls)
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        lengths = []
        for cnt in contours:
            # 1) FOCUS LOCK: Does this wall approach the robot by at least 5 meters?
            # If it doesn't (e.g., if it's a huge hangar wall outside), reject it immediately!
            distances = np.sqrt((cnt[:,0,0] - robot_px)**2 + (cnt[:,0,1] - robot_py)**2)
            min_dist_meters = np.min(distances) * resolution
            
            if min_dist_meters > 5.0:
                continue # This wall is too far from the robot (another room, etc.), ignore it!
                
            # 2) LENGTH DETECTION (UNIVERSAL CONTOUR APPROXIMATION/SMOOTHING METHOD)
            epsilon = 0.005 * cv2.arcLength(cnt, True)
            approx_contour = cv2.approxPolyDP(cnt, epsilon, True)
            
            perimeter = cv2.arcLength(approx_contour, True)
            length_pixels = perimeter / 2.0
            length_meters = length_pixels * resolution
            
            # 3) MINIMUM SIZE LOCK (Filter out pebbles and noise)
            if length_meters > 1.0:
                lengths.append(length_meters)
                
        if len(lengths) < 2:
            self.get_logger().info("Could not detect at least 2 distinct walls within a 5-meter radius of the robot... Keep moving...")
            return
            
        lengths.sort(reverse=True)
        
        wall1 = lengths[0]
        wall2 = lengths[1]
        average_len = (wall1 + wall2) / 2.0
        
        self.get_logger().info(f"--- FOCUSED MEASUREMENT --- | 1st Wall: {wall1:.2f}m | 2nd Wall: {wall2:.2f}m => NET AVERAGE: {average_len:.2f} METERS")

def main(args=None):
    rclpy.init(args=args)
    node = MapWallMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()