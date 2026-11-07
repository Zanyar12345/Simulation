#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray
from sensor_msgs.msg import JointState


class Publisher(Node):

    def __init__(self):
        super().__init__('Publisher')

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steer_position_controller/commands',
            10
        )
        
        self.susp_pub = self.create_publisher(
            Float64MultiArray,
            '/suspension_effort_controller/commands',
            10
        )

        self.slowness = -1/2
        self.vel = [0.0, 0.0, 0.0, 0.0]
        self.ang = [0.0, 0.0, 0.0, 0.0]

        self.sag_pos = 0.0
        self.sol_pos = 0.0
        
        self.sag_vel = 0.0
        self.sol_vel = 0.0

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.create_subscription(Int32, 'wheel_front_left_speed',  self.fl_speed_cb, 10)
        self.create_subscription(Int32, 'wheel_front_right_speed', self.fr_speed_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_left_speed',   self.rl_speed_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_right_speed',  self.rr_speed_cb, 10)

        self.create_subscription(Int32, 'wheel_front_left_angle',  self.fl_angle_cb, 10)
        self.create_subscription(Int32, 'wheel_front_right_angle', self.fr_angle_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_left_angle',   self.rl_angle_cb, 10)
        self.create_subscription(Int32, 'wheel_rear_right_angle',  self.rr_angle_cb, 10)

        self.create_timer(0.02, self.publish_commands)   

    def fl_speed_cb(self, msg): 
        self.vel[2] = float(msg.data)*(self.slowness)
    def fr_speed_cb(self, msg): 
        self.vel[3] = float(msg.data)*(self.slowness)
    def rr_speed_cb(self, msg): 
        self.vel[1] = float(msg.data)*(self.slowness)
    def rl_speed_cb(self, msg): 
        self.vel[0] = float(msg.data)*(self.slowness)

    def fl_angle_cb(self, msg): 
        self.ang[2] = (180-msg.data)*1.57/90
    def fr_angle_cb(self, msg): 
        self.ang[3] = (180-msg.data)*1.57/90
    def rr_angle_cb(self, msg): 
        self.ang[0] = (180-msg.data)*1.57/90
    def rl_angle_cb(self, msg): 
        self.ang[1] = (180-msg.data)*1.57/90

    def joint_state_cb(self, msg):
        try:
            if 'base_to_sag_legmil' in msg.name and 'base_to_sol_legmil' in msg.name:
                sag_idx = msg.name.index('base_to_sag_legmil')
                sol_idx = msg.name.index('base_to_sol_legmil')
                self.sag_pos = msg.position[sag_idx]
                self.sol_pos = msg.position[sol_idx]
                self.sag_vel = msg.velocity[sag_idx]
                self.sol_vel = msg.velocity[sol_idx]
        except ValueError:
            pass

    def publish_commands(self):

        vel_msg = Float64MultiArray()
        ang_msg = Float64MultiArray()

        vel_msg.data = self.vel
        ang_msg.data = self.ang

        self.wheel_pub.publish(vel_msg)
        self.steer_pub.publish(ang_msg)

        error = self.sag_pos + self.sol_pos
        error_dot = self.sag_vel + self.sol_vel
        
        p_gain = 50.0
        d_gain = 10.0

        effort = -(p_gain * error) - (d_gain * error_dot)
        
        susp_msg = Float64MultiArray()
        susp_msg.data = [effort, effort]
        self.susp_pub.publish(susp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
