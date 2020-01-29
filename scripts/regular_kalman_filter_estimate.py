#! /usr/bin/python

import rospy
import sys

from regular_kalman_filter.rosbot import Rosbot

if __name__ == "__main__":
    debug_mode = int(sys.argv[1])
    target_distance = float(sys.argv[2])
    robot_sensor = int(sys.argv[3])
    scan_sensor = int(sys.argv[4])

    log_level_mode = rospy.INFO
    if 1 == debug_mode:
        log_level_mode = rospy.DEBUG

    rospy.init_node("husarion_regular_kalman_filter", anonymous=False, log_level=log_level_mode)

    rosbot = Rosbot(robot_sensor, scan_sensor, target_distance)

    rospy.logdebug("Husarion ROSBot 2.0 regular Kalman filter initialized.")

    rospy.spin()
