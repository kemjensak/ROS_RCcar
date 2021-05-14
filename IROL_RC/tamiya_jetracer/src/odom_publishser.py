#!/usr/bin/env python3
import rospy
from pyvesc import VESC
import time
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import geometry_msgs.msg 
from tamiya_jetracer.msg import vesc_cmd
from math import *
import roslib
import tf
import turtlesim.msg


# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port = '/dev/ttyACM0'

meter_per_pulse = 0.004166666
meter_per_rotate = 0.205
wheel_rpm_per_speed = 116.144/9
ms_per_speed = (meter_per_rotate * wheel_rpm_per_speed) / 60
speed_per_ms = 1 / ms_per_speed
wheelbase = 0.257

x, y, theta = 0.0

steer_offset = 0

speed_cmd = 0
steer_cmd = 50 + steer_offset



def callback(data):
    global speed_cmd, steer_cmd
    speed_cmd = data.linear.x
    steer_cmd = degrees(atan(wheelbase / data.linear.x * data.angular.z)) 

def set_and_get_speed(speed, steer, count):
    while True:
        try:
            if count == 0:
                return "Err"
            # rospy.sleep(0.01)
            motor.set_servo((steer + steer_offset) / 100)
            rospy.sleep(0.01)
            motor.set_rpm(int(-speed*100))
            # motor.set_duty_cycle(speed/100)
            rospy.sleep(0.01)
            tacho = motor.get_measurements().tachometer
            if not (tacho == 0 or None):
                break

        except:
            count -= 1
            set_and_get_speed(speed, steer, count)


    return tacho

def handle_robot_pose(name):
    # global x, y, theta
    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                    tf.transformations.quaternion_from_euler(0, 0, theta),
                    rospy.Time.now(),
                    name,
                    "scanmatch_odom")


if __name__ == '__main__':

    rospy.init_node('vesc', anonymous=True)
    pub =  rospy.Publisher('/vesc_feedback', Int16, queue_size=10)
    
    rospy.Subscriber("/cmd_vel",geometry_msgs.msg.Twist ,callback)
    
    
    rate = rospy.Rate(20)

    motor = VESC(serial_port=serial_port)


    while not rospy.is_shutdown():
        
        # print(speed_cmd)
        
        # rospy.sleep(0.01)
        encoder = set_and_get_speed(speed_per_ms, steer_cmd, 10)
        print(ms_per_speed)
        print(encoder)
        
        rate.sleep()
        
    
    motor.set_rpm(0)
    motor.stop_heartbeat()


        
        











