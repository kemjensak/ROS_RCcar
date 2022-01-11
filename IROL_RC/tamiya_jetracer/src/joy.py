#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from math import sin, cos, pi

speed = 0
  
def callback(data):
    if data.axes[1] > 0:
        #motor.set_rpm(int(data.axes[1]*-3000-40000))
        motor.set_rpm(int(data.axes[1]*-3000-500))
    elif data.axes[1] == 0:
        motor.set_rpm(0)
        
    else:
        #motor.set_rpm(int(data.axes[1]*-3000+40000))
        motor.set_rpm(int(data.axes[1]*-3000+500))

    motor.set_servo(float(data.axes[2]/4.5*-1+0.326))
    #speed = motor.get_measurements().rpm
    #pub.publish(motor.get_measurements().rpm / 60 * 0.065 * pi)

    


rospy.init_node("control")
rospy.Subscriber("joy",Joy,callback)
pub = rospy.Publisher('speed',Int16,queue_size=10)

while not rospy.is_shutdown():
    
    try:
        speed = motor.get_measurements().rpm
        if abs(speed) <=20:
            speed = 0

    except AttributeError as e:
    	speed = speed
   
    
    print(speed)
    pub.publish(int(speed))
        #pub.publish(speed / 60 * 0.065 * pi)
    #rospy.spin()


