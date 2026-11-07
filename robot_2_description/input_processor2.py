#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


JOY_DEADZONE = 0.15
JOY_MAX_SPEED = 64    # duz surus tavan hizi
JOY_TURN_SPEED = 32   # donus tavan hizi
JOY_MIN_TURN = 20     # yerinde donuste minimum komut (kalkis icin)

NAV_MAX_SPEED = 50    # tekerlek komutu tavani
NAV_MIN_SPEED = 20    # hareket varsa en hizli taraf en az bu kadar olsun 
NAV_DEADBAND = 2.0    # bundan kucuk komutlar sifir sayilir 
NAV_TRACK_SCALE = 1.0 # yerinde donus yetersizse artir 
CMD_TIMEOUT = 0.5     # s, /cmd_vel bu kadar gelmezse dur


class HotasProcessor(Node):
    def __init__(self):
        super().__init__('hotas_control')

        self.declare_parameter('frequency', 50.0)
        self.frequency = self.get_parameter('frequency').value

        self.light_mode = False
        self.light_button = 5
        self.last_light_state = False

        self.rover_width = 82  
        self.rover_length = 95  

        self.publishers_ = {
            'wheel_front_left_speed':  self.create_publisher(Int32, 'wheel_front_left_speed', 10),
            'wheel_front_right_speed': self.create_publisher(Int32, 'wheel_front_right_speed', 10),
            'wheel_rear_left_speed':   self.create_publisher(Int32, 'wheel_rear_left_speed', 10),
            'wheel_rear_right_speed':  self.create_publisher(Int32, 'wheel_rear_right_speed', 10),
            'wheel_light':  self.create_publisher(Bool,  'wheel_light', 10),
            'sallama':      self.create_publisher(Int32, 'sallama', 10),
            'sallama2':     self.create_publisher(Int32, 'sallama2', 10)
        }
        self.msgs = {n: (Bool() if n == 'wheel_light' else Int32())
                     for n in self.publishers_}

        self.create_subscription(Joy, 'joy', self.joy_callrear, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callrear, 10)
        self.create_subscription(String, '/control_mode', self.mode_callback, 10)

        self.joy_msg = Joy()
        self.cmd_vel_msg = Twist()
        self.last_cmd_time = None

        self.control_mode = "NAV"

        self.create_timer(1.0 / self.frequency, self.timer_callrear)
        self.get_logger().info(f'Motor Control Node initialized at {self.frequency} Hz')

    
    def mode_callback(self, msg):
        if msg.data in ("NAV", "SPIN"):
            self.control_mode = "NAV"
        elif msg.data == "JOY":
            self.control_mode = "JOY"
        self.get_logger().info(f'Control mode: {self.control_mode}')

    def cmd_vel_callrear(self, msg):
        self.cmd_vel_msg = msg
        self.last_cmd_time = self.get_clock().now()

    def joy_callrear(self, msg):
        self.joy_msg = msg
        if len(msg.buttons) > self.light_button:
            state = bool(msg.buttons[self.light_button])
            if state and not self.last_light_state:
                self.light_mode = not self.light_mode
            self.last_light_state = state

    
    def shape_joy(self, x, deadzone=JOY_DEADZONE, power=2.0):

        if abs(x) < deadzone:
            return 0.0
        s = (abs(x) - deadzone) / (1.0 - deadzone)
        s = s ** power
        return s if x > 0 else -s

    def cmd_fresh(self):
        if self.last_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        return age < CMD_TIMEOUT

    def compute_joy(self):
        if len(self.joy_msg.axes) < 2:
            return 0.0, 0.0, False

        v = self.shape_joy(self.joy_msg.axes[1], power=2.0)   
        w = self.shape_joy(self.joy_msg.axes[0], power=1.0)   
        active = (v != 0.0 or w != 0.0)

        left  = v * JOY_MAX_SPEED - w * JOY_TURN_SPEED
        right = v * JOY_MAX_SPEED + w * JOY_TURN_SPEED

        if v == 0.0 and w != 0.0:
            turn = max(abs(w) * JOY_TURN_SPEED, JOY_MIN_TURN)
            right = turn if w > 0 else -turn
            left = -right

        cap = max(JOY_MAX_SPEED, JOY_TURN_SPEED)
        m = max(abs(left), abs(right))
        if m > cap:
            left, right = left * cap / m, right * cap / m
        return left, right, active

    def compute_nav(self):
        v = self.cmd_vel_msg.linear.x * 100.0
        half_track = (self.rover_width / 2.0) * NAV_TRACK_SCALE
        w = self.cmd_vel_msg.angular.z * half_track

        left  = v - w
        right = v + w

        m = max(abs(left), abs(right))
        if m < NAV_DEADBAND:
            return 0.0, 0.0
        if m < NAV_MIN_SPEED:            
            k = NAV_MIN_SPEED / m
            left, right, m = left * k, right * k, NAV_MIN_SPEED
        if m > NAV_MAX_SPEED:           
            k = NAV_MAX_SPEED / m
            left, right = left * k, right * k
        return left, right

    def timer_callrear(self):
        joy_left, joy_right, joy_active = self.compute_joy()

        use_nav = (self.control_mode == "NAV"
                   and not joy_active        
                   and self.cmd_fresh())    

        if use_nav:
            left, right = self.compute_nav()
            src = 'NAV'
        else:
            left, right = joy_left, joy_right
            src = 'JOY'

        self.msgs['sallama'].data = 18
        self.msgs['sallama2'].data = 81
        for wheel in ('front_left', 'rear_left'):
            self.msgs[f'wheel_{wheel}_speed'].data = int(left)
        for wheel in ('front_right', 'rear_right'):
            self.msgs[f'wheel_{wheel}_speed'].data = int(right)
        self.msgs['wheel_light'].data = self.light_mode

        # self.get_logger().info(f'{src}: L={int(left)} R={int(right)}',
        #                        throttle_duration_sec=1.0)

        for name, publisher in self.publishers_.items():
            publisher.publish(self.msgs[name])


def main(args=None):
    rclpy.init(args=args)
    node = HotasProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()