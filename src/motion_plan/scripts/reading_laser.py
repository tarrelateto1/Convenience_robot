#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan


def clbk_laser(msg):
    # 1000 / 5 = 200
    regions = [
        min(min(msg.ranges[0:199]), 10),
        min(min(msg.ranges[200:399]), 10),
        min(min(msg.ranges[400:599]), 10),
        min(min(msg.ranges[600:799]), 10),
        min(min(msg.ranges[800:999]), 10),
    ]
    rospy.loginfo(regions)


def main():
    rospy.init_node("reading_laser")

    sub = rospy.Subscriber("/robot/laser/scan", LaserScan, clbk_laser)

    rospy.spin()


if __name__ == "__main__":
    main()
