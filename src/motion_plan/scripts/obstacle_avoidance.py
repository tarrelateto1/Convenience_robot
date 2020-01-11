#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None


def clbk_laser(msg):
    """
        We need to provide a callback function 
        to the Subscriber defined in main, 
        for this purpose we have this function. 
        It receives laser scan data comprising 
        of 720 readings and converts it into 5 readings 
    """
    regions = {
        "right": min(min(msg.ranges[500:699]), 10),
        "fright": min(min(msg.ranges[700:899]), 10),
        "front": min(min(msg.ranges[900:1099]), 10),
        "fleft": min(min(msg.ranges[1100:1299]), 10),
        "left": min(min(msg.ranges[1300:1499]), 10),
    }

    take_action(regions)


def take_action(regions):
    """
        This function implements the obstacle avoidance logic. 
        Based on the distances sensed in the five region 
        (left, center-left, center, center-right, right). 
        We consider possible combinations for obstacles, 
        once we identify the obstacle configuration 
        we steer the robot away from obstacle.
    """
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ""

    if regions["front"] > 1 and regions["fleft"] > 1 and regions["fright"] > 1:
        state_description = "case 1 - nothing"
        linear_x = 0.4
        angular_z = 0
    elif regions["front"] < 1 and regions["fleft"] > 1 and regions["fright"] > 1:
        state_description = "case 2 - front"
        linear_x = 0
        angular_z = -0.5
    elif regions["front"] > 1 and regions["fleft"] > 1 and regions["fright"] < 1:
        state_description = "case 3 - fright"
        linear_x = 0
        angular_z = -0.5
    elif regions["front"] > 1 and regions["fleft"] < 1 and regions["fright"] > 1:
        state_description = "case 4 - fleft"
        linear_x = 0
        angular_z = 0.5
    elif regions["front"] < 1 and regions["fleft"] > 1 and regions["fright"] < 1:
        state_description = "case 5 - front and fright"
        linear_x = 0
        angular_z = -0.5
    elif regions["front"] < 1 and regions["fleft"] < 1 and regions["fright"] > 1:
        state_description = "case 6 - front and fleft"
        linear_x = 0
        angular_z = 0.5
    elif regions["front"] < 1 and regions["fleft"] < 1 and regions["fright"] < 1:
        state_description = "case 7 - front and fleft and fright"
        linear_x = 0
        angular_z = -0.5
    elif regions["front"] > 1 and regions["fleft"] < 1 and regions["fright"] < 1:
        state_description = "case 8 - fleft and fright"
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = "unknown case"
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():
    """
        This is the entry point of the file. 
        This function sets up a Subscriber 
        to the laser scan topic /m2wr/laser/scan 
        and a Publisher to /cmd_vel topic.
    """
    global pub

    rospy.init_node("reading_laser")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    sub = rospy.Subscriber("/m2wr/laser/scan", LaserScan, clbk_laser)

    rospy.spin()


if __name__ == "__main__":
    main()
