#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

def main():
    rospy.init_node("path_once_plotter", anonymous=True)

    # Wait for one message in the main thread
    rospy.loginfo("Waiting for /mm_planner/path message...")
    msg = rospy.wait_for_message("/mm_planner/path", Path)
    rospy.loginfo("Message received. Plotting...")

    # Extract x, y
    xs = [pose.pose.position.x for pose in msg.poses]
    ys = [pose.pose.position.y for pose in msg.poses]

    # Plot in main thread
    plt.figure(figsize=(10, 6))
    plt.plot(xs, ys, 'b-', marker='o', label="/mm_planner/path")
    plt.title("Path from /mm_planner/path")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
