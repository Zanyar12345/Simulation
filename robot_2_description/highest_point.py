#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import math

def euler_from_quaternion(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z 

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class WaypointNavigatorAndHighestFinder(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_highest_finder')
        
        map_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(PointCloud2, '/cloud_map', self.pointcloud_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)

        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.navigator = BasicNavigator()
        
        self.waypoints = [
            (10.0, 11.0),
            (17.0, -5.0),
            (-1.0, 8.0)
        ]
        
        self.spin_waypoint_index = 1 
        
        self.current_wp_index = 0
        self.stage = "INIT"
        self.goal_sent = False
        
        self.local_max_z = -float('inf')
        self.local_target_x = 0.0
        self.local_target_y = 0.0
        self.last_frame_id = "map"
        
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.accumulated_yaw = 0.0
        self.last_yaw = 0.0
        self.spin_started = False
        
        self.create_timer(0.5, self.timer_callback)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_yaw = yaw
    
        if self.stage == "SPIN" and self.spin_started:
            dyaw = normalize_angle(self.current_yaw - self.last_yaw)
            self.accumulated_yaw += dyaw
            self.last_yaw = self.current_yaw

    def pointcloud_callback(self, msg):
        if self.stage != "SPIN":
            return

        self.last_frame_id = msg.header.frame_id
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[0], point[1], point[2]
            if 0.3 < z < 3.0: 
                if z > self.local_max_z:
                    self.local_max_z = z
                    self.local_target_x = x
                    self.local_target_y = y

    def go_to_pose(self, x, y, frame_id="map"):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0 
        goal_pose.pose.orientation.w = 1.0 
        self.navigator.goToPose(goal_pose)
        self.get_logger().info(f"Navigating to target: X={x:.2f}, Y={y:.2f}")

    def timer_callback(self):
        if self.stage == "INIT":
            self.get_logger().info("Waypoint route starting.")
            self.stage = "NAVIGATE_WAYPOINTS"
            
        elif self.stage == "NAVIGATE_WAYPOINTS":
            if not self.goal_sent:
                if self.current_wp_index < len(self.waypoints):
                    wx, wy = self.waypoints[self.current_wp_index]
                    self.get_logger().info(f"Navigating to waypoint {self.current_wp_index + 1}/{len(self.waypoints)}.")
                    self.go_to_pose(wx, wy)
                    self.goal_sent = True
                else:
                    self.get_logger().info("All waypoints completed!")
                    self.stage = "DONE"
            else:
                if self.navigator.isTaskComplete():
                    self.goal_sent = False
                    self.get_logger().info(f"Waypoint {self.current_wp_index + 1} completed.")
                    
                    if self.current_wp_index == self.spin_waypoint_index:
                        self.get_logger().info("Designated point reached. Starting 360-degree rotation.")
                        self.stage = "SPIN"
                        self.spin_started = False
                        self.local_max_z = -float('inf')
                    
                    self.current_wp_index += 1
                    
        elif self.stage == "SPIN":
            if not self.spin_started:
                self.start_yaw = self.current_yaw
                self.last_yaw = self.current_yaw
                self.accumulated_yaw = 0.0
                self.spin_started = True
                
                msg_mode = String()
                msg_mode.data = "SPIN"
                self.mode_pub.publish(msg_mode)
            
            twist = Twist()
            twist.angular.z = 0.5 
            self.cmd_pub.publish(twist)
            
            if abs(self.accumulated_yaw) >= (2.0 * math.pi) - 0.05: 
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.spin_started = False
                
                self.get_logger().info("360-degree rotation completed.")
                
                msg_mode = String()
                msg_mode.data = "NAV"
                self.mode_pub.publish(msg_mode)
                
                if self.local_max_z != -float('inf'):
                    self.get_logger().info(f"New highest point found! Z={self.local_max_z:.2f}. Navigating there.")
                    self.go_to_pose(self.local_target_x, self.local_target_y, self.last_frame_id)
                    self.stage = "NAVIGATE_HIGHEST"
                else:
                    self.get_logger().info("No new object found during rotation. Continuing to remaining waypoints.")
                    self.stage = "NAVIGATE_WAYPOINTS"
                    
        elif self.stage == "NAVIGATE_HIGHEST":
            if self.navigator.isTaskComplete():
                self.get_logger().info("Highest object reached! Continuing to remaining waypoints if any.")
                self.stage = "NAVIGATE_WAYPOINTS"
                
        elif self.stage == "DONE":
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorAndHighestFinder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()