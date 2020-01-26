#! /usr/bin/python

import rospy
import sys

from pa2.rosbot import Rosbot

if __name__ == "__main__":
    robot_sensor = int(sys.argv[1])
    scan_sensor = int(sys.argv[2])

    rospy.init_node("rosbot8_simple_kalman_filter", anonymous=False)

    rosbot = Rosbot(robot_sensor, scan_sensor)

    rospy.loginfo("Rosbot8 control initialized.")

    rospy.spin()
