#!/usr/bin/env python3
import rclpy as rcl
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import math

class four_wheel(Node):
    def __init__(self):
        super().__init__("four_wheel")

    
        self.L = 1.13  
        self.W = 0.84  
        
        self.speed = 63.0 
        
        self.create_subscription(Joy, "joy", self.back, 10)
        
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/steer_position_controller/commands', 10)
        
        self.get_logger().info("Başladı")

    def back(self, msg: Joy):
        
        steer_msg = Float64MultiArray()
        wheel_msg = Float64MultiArray()

        if abs(msg.axes[0]) < 0.05:

            speed = msg.axes[1] * self.speed
            steer_msg.data = [0.0, 0.0, 0.0, 0.0]
            wheel_msg.data = [speed, speed, speed, speed]
        

        else:
            delta = msg.axes[0]*(math.pi/2)
            if  delta<=math.atan(-self.L/self.W) or delta>=math.atan(self.L/self.W):
                if delta>0:
                    delta=math.atan(self.L/self.W)
                else:
                    delta=math.atan(-self.L/self.W)
            else:
                delta = msg.axes[0]*(math.pi/2)
                
            R = (self.L / 2.0) / math.tan(abs(delta))
            

            if msg.axes[0] > 0:
                center_y = R  
            else:
                center_y = -R 



            dist_y_left = center_y - (self.W / 2.0)

            dist_y_right = center_y - (-self.W / 2.0)


            radius_fl = math.hypot(self.L / 2.0, dist_y_left)
            radius_rl = math.hypot(-self.L / 2.0, dist_y_left)
            
            radius_fr = math.hypot(self.L / 2.0, dist_y_right)
            radius_rr = math.hypot(-self.L / 2.0, dist_y_right)

            angle_fl = math.atan((self.L / 2.0) / dist_y_left)
            angle_rl = math.atan((-self.L / 2.0) / dist_y_left) 
            
            angle_fr = math.atan((self.L / 2.0) / dist_y_right)
            angle_rr = math.atan((-self.L / 2.0) / dist_y_right) 

            max_radius = max(radius_fl, radius_fr, radius_rl, radius_rr)
            
            base_speed = msg.axes[1] * self.speed
            
            v_fl = base_speed * (radius_fl / max_radius)
            v_fr = base_speed * (radius_fr / max_radius)
            v_rl = base_speed * (radius_rl / max_radius)
            v_rr = base_speed * (radius_rr / max_radius)

            steer_msg.data = [angle_fl, angle_fr, angle_rr, angle_rl]
            wheel_msg.data = [v_fl, v_fr, v_rr, v_rl]



        self.steer_pub.publish(steer_msg)
        self.wheel_pub.publish(wheel_msg)

def main(args=None):
    rcl.init(args=args)
    node = four_wheel()
    rcl.spin(node)
    node.destroy_node()
    rcl.shutdown()

if __name__ == "__main__":
    main()