#!/usr/bin/env python3
import rospy, geometry_msgs.msg, nav_msgs.msg
from jetracer.nvidia_racecar import NvidiaRacecar
import time

car = NvidiaRacecar()
car.throttle_gain = 0.5
car.steering_offset = 0.02
car.steering = 0.0
car.throttle = 0.0

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo(msg.linear.y)
    rospy.loginfo(msg.linear.x)

    rospy.loginfo(msg.angular.z)

    
     #car.throttle = - msg.linear.x + 0.28
    if msg.linear.x > 0.015:
        car.throttle = - 0.178 + 0.28
        if msg.linear.x > 2:
          car.throttle = - 0.18 + 0.28
    elif msg.linear.x > 0:
        car.throttle = 0.28
        car.steering = 0.0
    elif msg.linear.x < 0:
        car.throttle = 0.42
    if msg.linear.x > 0:
        car.steering = msg.angular.z * 0.5
    elif msg.linear.x < 0:
        car.steering = msg.angular.z * -0.5

    
rospy.init_node('tamiya_controller')

sub = rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, callback)

rospy.spin()









