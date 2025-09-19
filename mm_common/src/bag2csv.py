#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import csv
import math
import os
import matplotlib.pyplot as plt
import signal
import sys

class GNSSLogger:
    def __init__(self):
        rospy.init_node('gnss_logger', anonymous=True)

        self.csv_dir = os.path.expanduser('~/catkin_ws/src/heven_mm_2025/mm_common/csv')
        if not os.path.exists(self.csv_dir):
            os.makedirs(self.csv_dir)
        self.csv_path = os.path.join(self.csv_dir, 'gps_log.csv')

        self.csv_file = open(self.csv_path, mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        # self.csv_writer.writerow(['Latitude', 'Longitude'])

        self.prev_x = []
        self.prev_y = []
        self.origin_lat = None

        # rospy.Subscriber('/carla/ego_vehicle/gnss', NavSatFix, self.gnss_callback)
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gnss_callback)
        rospy.loginfo("GNSS Logger Started. Saving to: %s", self.csv_path)

        # Handle shutdown to plot at the end
        signal.signal(signal.SIGINT, self.handle_shutdown)

        rospy.spin()

    def gnss_callback(self, data):
        lat = data.latitude
        lon = data.longitude

        if self.origin_lat is None:
            self.origin_lat = lat

        x = lon * math.cos(self.origin_lat * math.pi / 180) * math.pi * 6378.135 / 180 * 1000
        y = lat * math.pi * 6378.135 / 180 * 1000

        if len(self.prev_x) == 0 or len(self.prev_y) == 0:
            self.log_position(lat, lon, x, y)
            return

        distance = math.sqrt((x - self.prev_x[-1])**2 + (y - self.prev_y[-1])**2)
        if distance >= 1.0:
            self.log_position(lat, lon, x, y)

    def log_position(self, lat, lon, x, y):
        self.csv_writer.writerow([lat, lon])
        self.csv_file.flush()
        self.prev_x.append(x)
        self.prev_y.append(y)
        rospy.loginfo(f"Logged: lat={lat}, lon={lon}, x={x:.2f}, y={y:.2f}")

    def handle_shutdown(self, sig, frame):
        rospy.loginfo("Shutting down... plotting x/y path.")
        self.csv_file.close()
        self.plot_xy_path()
        sys.exit(0)

    def plot_xy_path(self):
        xs = self.prev_x
        ys = self.prev_y

        plt.figure(figsize=(10, 6))
        plt.plot(xs, ys, marker='o', linestyle='-', color='green')
        plt.title('GNSS Path in Local Meters')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.grid(True)
        plt.axis('equal')
        plt.show()


if __name__ == '__main__':
    try:
        GNSSLogger()
    except rospy.ROSInterruptException:
        pass