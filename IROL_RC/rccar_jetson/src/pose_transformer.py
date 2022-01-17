#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped

from math import *

rospy.init_node('pose_transformer', anonymous=True)

def poseCallback(msg):
    poseStpd = PoseStamped()
    poseStpd.header= msg.header
    poseStpd.pose.position = msg.pose.pose.position
    poseStpd.pose.orientation = msg.pose.pose.orientation
    currnetPosePublisher.publish(poseStpd)

currnetPosePublisher = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped ,poseCallback) # auto


rospy.spin()
