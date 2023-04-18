#~/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "X: %s", data.point.x)
    rospy.loginfo(rospy.get_caller_id() + "Y: %s", data.point.y)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('clicked_point', PointStamped, callback)
    # rospy.Subscriber('cmd_vel', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()