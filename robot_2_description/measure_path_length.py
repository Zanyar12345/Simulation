#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math

class PathLengthCalculator(Node):
    def __init__(self):
        super().__init__('path_length_calculator')
        # Nav2 varsayılan olarak global rotayı /plan veya /received_global_plan üzerinde yayınlar.
        # Bu script /plan topic'ini dinler.
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10)
        self.get_logger().info('Kıvrımlı rota ölçücü başlatıldı! Lütfen RViz2 üzerinden tünelin çıkışına bir hedef (Nav2 Goal) gönderin.')

    def path_callback(self, msg):
        total_length = 0.0
        poses = msg.poses
        if len(poses) < 2:
            return

        for i in range(len(poses) - 1):
            x1 = poses[i].pose.position.x
            y1 = poses[i].pose.position.y
            x2 = poses[i+1].pose.position.x
            y2 = poses[i+1].pose.position.y
            
            dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            total_length += dist

        self.get_logger().info(f'\n======================================================\n Tünelin (Kıvrımlı Rotanın) Toplam Uzunluğu: {total_length:.2f} METRE\n======================================================')

def main(args=None):
    rclpy.init(args=args)
    node = PathLengthCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
