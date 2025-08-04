#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Int32, Int32MultiArray
import math

PI = math.pi

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = (math.degrees(yaw) + 360) % 360
    return (360 - yaw_deg) % 360

def deg2rad(deg): return deg * PI / 180
def rad2deg(rad): return rad * 180 / PI

def get_bearing(lat1, lon1, lat2, lon2):
    lat1_rad = deg2rad(lat1)
    lat2_rad = deg2rad(lat2)
    dlon_rad = deg2rad(lon2 - lon1)
    y = math.sin(dlon_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    bearing = rad2deg(math.atan2(y, x))
    return (bearing + 360) % 360

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # meters
    dlat = deg2rad(lat2 - lat1)
    dlon = deg2rad(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

class WaypointHeadingNode(Node):
    def __init__(self):
        super().__init__('waypoint_heading_node')

        # QoS 설정
        gps_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # 목표 지점 목록
        self.waypoint_list = [
            (35.0694199, 128.5789888),  # 1번
            (35.0695100, 128.5791000)   # 2번
        ]
        self.current_goal_idx = 0
        self.current_yaw = None

        # 퍼블리셔
        self.heading_pub = self.create_publisher(Int32, '/waypoint/heading', 10)
        self.status_pub = self.create_publisher(Int32MultiArray, '/waypoint/status', 10)
        self.goal_pub = self.create_publisher(NavSatFix, '/waypoint/goal', 10)

        # 구독자
        self.create_subscription(Imu, '/imu/data', self.imu_callback, imu_qos)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, gps_qos)

        self.get_logger().info("🧭 WaypointHeadingNode launched.")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def gps_callback(self, msg: NavSatFix):
        if self.current_goal_idx >= len(self.waypoint_list):
            return  # 모든 waypoint 도달 완료

        current_lat = msg.latitude
        current_lon = msg.longitude
        goal_lat, goal_lon = self.waypoint_list[self.current_goal_idx]

        # 현재 목표 좌표 퍼블리시
        self.publish_current_goal(goal_lat, goal_lon)

        # 거리 계산
        distance = haversine(current_lat, current_lon, goal_lat, goal_lon)

        # 도달 여부 확인
        if distance < 10:
            self.publish_status(self.current_goal_idx + 1, 1)  # 도달
            self.get_logger().info(f"🎯 Waypoint {self.current_goal_idx + 1} 도달")
            self.current_goal_idx += 1
            return

        if self.current_yaw is None:
            self.get_logger().warn("⏳ IMU yaw not yet received.")
            return

        bearing = get_bearing(current_lat, current_lon, goal_lat, goal_lon)
        delta = (bearing - self.current_yaw + 540) % 360 - 180
        rel_heading = int(round(90 - delta))
        rel_heading = max(0, min(rel_heading, 180))

        self.publish_heading(rel_heading)
        self.publish_status(self.current_goal_idx + 1, 0)

        self.get_logger().info(
            f"[Waypoint {self.current_goal_idx+1}] 📍dist={distance:.1f}m, bearing={bearing:.1f}°, yaw={self.current_yaw:.1f}°, heading={rel_heading}"
        )

    def publish_heading(self, heading):
        msg = Int32()
        msg.data = heading
        self.heading_pub.publish(msg)

    def publish_status(self, goal_number, status_code):
        msg = Int32MultiArray()
        msg.data = [goal_number, status_code]
        self.status_pub.publish(msg)

    def publish_current_goal(self, lat, lon):
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
