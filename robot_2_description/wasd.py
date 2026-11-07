#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard

class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard")
        self.pub = self.create_publisher(Joy, 'joy', 10)
        
        self.speed = 0.0
        self.angle = 0.0
        self.was_active = False
        self.spin_active = 0
        
        self.MAX_SPEED = 1.0
        self.MIN_SPEED = -1.0
        self.MAX_ANGLE = 1.0
        self.MIN_ANGLE = -1.0

        self.ACCEL_STEP = 0.03
        self.REVERSE_STEP = 0.03
        self.SPEED_DECAY = 0.02

        self.STEER_STEP = 0.07
        self.ANGLE_DECAY = 0.06
            
        self.pressed_keys = set()
        
        self.msg = Joy()
        self.msg.buttons = [0, 0, 0, 0, 0, 0]
        self.msg.axes = [0.0, 0.0]

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        self.timer = self.create_timer(0.05, self.update_physics)

    def on_press(self, key):
        try:
            k = key.char.lower()
            self.pressed_keys.add(k)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            k = key.char.lower()
            self.pressed_keys.discard(k)
        except AttributeError:
            pass

    def update_physics(self):
        if 'q' in self.pressed_keys:
            self.speed = 0.0
            self.angle = 0.0
        # elif '1' in self.pressed_keys:
        #     self.spin_active == 0
        #     self.msg.buttons[2] = 1
        #     self.msg.buttons[4] = 0
        #     self.pub.publish(self.msg)
        #     self.spin_active = 1
        # elif "2" in self.pressed_keys:
        #     self.msg.buttons[2] = 0
        #     self.msg.buttons[4] = 1
        #     self.pub.publish(self.msg)
        #     self.spin_active = 0
        else:
            if 'w' in self.pressed_keys and 's' not in self.pressed_keys:
                self.speed = min(self.MAX_SPEED, self.speed + self.ACCEL_STEP)
            elif 's' in self.pressed_keys and 'w' not in self.pressed_keys:
                self.speed = max(self.MIN_SPEED, self.speed - self.REVERSE_STEP)
            else:
                if self.speed > 0:
                    self.speed = max(0.0, self.speed - self.SPEED_DECAY)
                elif self.speed < 0:
                    self.speed = min(0.0, self.speed + self.SPEED_DECAY)

            if 'a' in self.pressed_keys and 'd' not in self.pressed_keys:
                self.angle = min(self.MAX_ANGLE, self.angle + self.STEER_STEP)
            elif 'd' in self.pressed_keys and 'a' not in self.pressed_keys:
                self.angle = max(self.MIN_ANGLE, self.angle - self.STEER_STEP)
            else:
                if self.angle > 0:
                    self.angle = max(0.0, self.angle - self.ANGLE_DECAY)
                elif self.angle < 0:
                    self.angle = min(0.0, self.angle + self.ANGLE_DECAY)

        self.msg.axes[1] = float(self.speed / self.MAX_SPEED)
        self.msg.axes[0] = float(self.angle / self.MAX_ANGLE)
        self.pub.publish(self.msg)

        is_active = (self.speed != 0.0 or self.angle != 0.0 or bool(self.pressed_keys))
        if is_active or self.was_active:
            self.get_logger().info(f"Hız1: {self.speed:5.1f} | Açı: {self.angle:5.1f}")
        self.was_active = is_active

    def stop_rover(self):
        self.speed = 0.0
        self.angle = 0.0
        self.msg.axes = [0.0, 0.0]
        self.pub.publish(self.msg)
        self.get_logger().info(f"Hız: {self.speed:5.1f} | Açı: {self.angle:5.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()