import os
import math
import time
import threading

import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class GNSSHeadingNode(Node):

    def __init__(self):
        super().__init__('gnss_heading_node')

        print('dev update')
        device = os.getenv('RTK_DEVICE')
        self.ser = serial.Serial(device, 115200, timeout=1)
        self.odom_pub = self.create_publisher(Odometry, '/gnss/odom', 10)

        self.last_pos = None
        self.last_yaw = None

        thread = threading.Thread(target=self.reader_loop, daemon=True)
        thread.start()

    def reader_loop(self):
        while rclpy.ok():
            try:
                print('---')
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$'):
                    self.handle_nmea(line)
            except Exception as e:
                self.get_logger().error(f'NMEA read error: {e}')

    def handle_nmea(self, line):
        parts = line.split(',')
        print('Handler:', parts)

        if line.startswith('$GPGGA'):
            try:
                if parts[2] == '' or parts[4] == '' or parts[9] == '':
                    return
                lat = self.parse_lat(parts[2], parts[3])
                lon = self.parse_lon(parts[4], parts[5])
                alt = float(parts[9])
                self.last_pos = (lat, lon, alt)
            except Exception as e:
                print('pos fail:', e)
                return

        elif line.startswith('$GPHRP'):
            try:
                if parts[1] == '':
                    return
                yaw = float(parts[1])
                self.last_yaw = yaw
            except Exception as e:
                print('yaw fail (HRP):', e)
                return

        elif line.startswith('$GPHDT'):
            try:
                if parts[1] == '':
                    return
                yaw = float(parts[1])
                self.last_yaw = yaw
            except Exception as e:
                print('yaw fail (HDT):', e)
                return

        elif line.startswith('$PASHR'):
            try:
                if parts[2] == '':
                    return
                yaw = float(parts[2])
                self.last_yaw = yaw
            except Exception as e:
                print('yaw fail (PASHR):', e)
                return

        print('Publish: pos=', self.last_pos, 'yaw=', self.last_yaw)
        if self.last_pos and self.last_yaw is not None:
            self.publish_odom()

    def publish_odom(self):
        print('publishing')
        lat, lon, alt = self.last_pos
        yaw_rad = math.radians(self.last_yaw)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'

        # Dummy conversion – replace with UTM or ENU as needed
        odom.pose.pose.position.x = lon
        odom.pose.pose.position.y = lat
        odom.pose.pose.position.z = alt

        q = self.yaw_to_quaternion(yaw_rad)
        odom.pose.pose.orientation = q

        self.odom_pub.publish(odom)

    def yaw_to_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

    def parse_lat(self, val, hemi):
        deg = float(val[:2])
        min = float(val[2:])
        lat = deg + min / 60.0
        return lat if hemi == 'N' else -lat

    def parse_lon(self, val, hemi):
        deg = float(val[:3])
        min = float(val[3:])
        lon = deg + min / 60.0
        return lon if hemi == 'E' else -lon


def main():
    rclpy.init()
    node = GNSSHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
