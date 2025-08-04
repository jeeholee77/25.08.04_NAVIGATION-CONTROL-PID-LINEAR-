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
        self.prev_error = 0  # PID 상태 저장

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

        # PID 설정
        target_heading = 90
        error = target_heading - heading  # 판정된 방향: heading 가 크면 우회전

        # PID 계수
        Kp = 1.5
        Kd = 0.5

        P = Kp * error
        D = Kd * (error - self.prev_error)
        output = P + D

        self.prev_error = error

        # 중심 PWM 방지
        base_pwm = 1450
        max_output = 200
        output = max(-max_output, min(max_output, output))

        left_pwm = int(base_pwm + output)
        right_pwm = int(base_pwm - output)

        left_pwm = max(1200, min(1700, left_pwm))
        right_pwm = max(1200, min(1700, right_pwm))

        pwm_msg.data = [left_pwm, right_pwm]
        self.pwm_pub.publish(pwm_msg)

        self.get_logger().info(
            f"[PID] heading={heading}, error={error:.1f}, output={output:.1f}, L={left_pwm}, R={right_pwm}"
        )

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
