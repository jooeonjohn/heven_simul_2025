#!/usr/bin/env python3

import sys
import time
import rospy
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

from lib.network.UDP import Sender
from lib.define.EgoCtrlCmd import EgoCtrlCmd
from morai_msgs.msg import CtrlCmd

IP = '127.0.0.1'
PORT = 9093

def callback(msg, sender):
    """
    Convert ROS CtrlCmd message to UDP EgoCtrlCmd and send
    """
    data = EgoCtrlCmd()
    data.ctrl_mode = 2  # AutoMode (or map this from ROS if needed)
    data.gear = 4       # Example: D gear
    data.cmd_type = msg.longlCmdType

    # Map fields from ROS message
    data.accel = msg.accel
    data.brake = msg.brake
    data.steer = msg.steering
    
    print(data.steer)

    sender.send(data)

def main():
    rospy.init_node('ego_ctrl_udp_publisher', anonymous=False)
    sender = Sender(IP, PORT)

    # Subscribe to ROS topic and forward to UDP
    rospy.Subscriber('ctrl_cmd', CtrlCmd, callback, sender)

    rospy.loginfo(f"Subscribed to 'command' and sending UDP to {IP}:{PORT}")
    rospy.spin()  # keep node alive

if __name__ == '__main__':
    main()
