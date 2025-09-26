#!/usr/bin/env python

import rospy
from morai_msgs.msg import GPSMessage
import csv
import math
import os
import matplotlib.pyplot as plt
import signal
import sys
from geographic_msgs.msg import GeoPoint
import geodesy.utm

class GNSSLogger:
    def __init__(self):
        rospy.init_node('gnss_logger', anonymous=True)

        self.csv_dir = os.path.expanduser('~/catkin_ws/src/heven_simul_2025/mm_common/csv')
        if not os.path.exists(self.csv_dir):
            os.makedirs(self.csv_dir)
        self.csv_path = os.path.join(self.csv_dir, 'gps_log.csv')

        self.csv_file = open(self.csv_path, mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        # self.csv_writer.writerow(['Latitude', 'Longitude'])

        self.prev_x = []
        self.prev_y = []

        rospy.Subscriber('/gps', GPSMessage, self.gnss_callback)
        rospy.loginfo("GNSS Logger Started. Saving to: %s", self.csv_path)

        # Handle shutdown to plot at the end
        signal.signal(signal.SIGINT, self.handle_shutdown)

        rospy.spin()

    def gnss_callback(self, data):
        lat = data.latitude
        lon = data.longitude
        h = data.altitude
        x0 = data.eastOffset
        y0 = data.northOffset

        # Create a GeoPoint
        geo_point = GeoPoint()
        geo_point.latitude = lat
        geo_point.longitude = lon
        geo_point.altitude = h

        # Convert to UTM
        utm_point = geodesy.utm.fromMsg(geo_point)  # returns a UTMPoint

        # Compute ENU relative to reference offsets
        x = utm_point.easting - x0
        y = utm_point.northing - y0

        if len(self.prev_x) == 0 or len(self.prev_y) == 0:
            self.log_position(lat, lon, x, y)
            return

        distance = math.sqrt((x - self.prev_x[-1])**2 + (y - self.prev_y[-1])**2)
        if distance >= 1.0:
            self.log_position(lat, lon, x, y)

    def log_position(self, lat, lon, x, y):
        self.csv_writer.writerow([x, y])
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