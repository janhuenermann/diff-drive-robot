#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import Pose2D

rospy.init_node('pub_pose2')

argv = rospy.myargv(argv=sys.argv)

if len(argv) < 4:
    print("usage: pub_pose2.py topic x y")
    exit()

topic = argv[1]
x = float(argv[2])
y = float(argv[3])

pose = Pose2D()
pose.x = x
pose.y = y

pub = rospy.Publisher(topic, Pose2D, latch=True, queue_size=1)
pub.publish(pose)

rospy.loginfo("Published pose to " + topic)
rospy.spin()