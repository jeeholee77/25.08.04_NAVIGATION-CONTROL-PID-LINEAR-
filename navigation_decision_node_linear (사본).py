#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
import numpy as np

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

class NavigationDecisionNode(Node):
    def __init__(self):
        super().__init__('navigation_decision_node')

        self.b_heading = 90
        self.safe_heading_list = []

        self.pwm_pub = self.create_publisher(Int32MultiArray, '/motor/pwm_cmd', 10)

        self.final_heading_pub = self.create_publisher(Int32, '/final_heading', 10)

        self.motor_sub = self.create_subscription(
            Int32,
            '/waypoint/heading',
            self.heading_callback,
            10
        )

        self.safe_heading_sub = self.create_subscription(
            Int32MultiArray,
            '/safe_heading_list',
            self.safe_callback,
            10
        )

    def safe_callback(self, msg):
        self.safe_heading_list = msg.data

    def heading_callback(self, msg):
        self.b_heading = msg.data
        final_heading = self.select_heading(self.b_heading, self.safe_heading_list)
        self.get_logger().info(f'🎯 선택된 heading: {final_heading}')
        self.publish_final_heading(final_heading)
        self.motor_control(final_heading)

    def select_heading(self, desired, safe_list):
        if desired in safe_list:
            return desired
        elif not safe_list:
            return 90
        else:
            return find_nearest(safe_list, desired)

    def motor_control(self, heading):
        pwm_msg = Int32MultiArray()

        def lerp(x, x0, x1, y0, y1):
            return int(y0 + (y1 - y0) * (x - x0) / (x1 - x0))

        if heading <= 20:
            left = 1650
            right = 1365
        elif heading <= 80:
            left = lerp(heading, 20, 80, 1650, 1450)
            right = lerp(heading, 20, 80, 1365, 1450)
        elif heading <= 100:
            left = 1450
            right = 1450
        elif heading <= 160:
            left = lerp(heading, 100, 160, 1450, 1365)
            right = lerp(heading, 100, 160, 1450, 1650)
        else:
            left = 1365
            right = 1650

        pwm_msg.data = [left, right]
        self.pwm_pub.publish(pwm_msg)

    def publish_final_heading(self, heading):
        msg = Int32()
        msg.data = heading
        self.final_heading_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
