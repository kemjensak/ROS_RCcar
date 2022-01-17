#!/usr/bin/env python3
import rospy
from pyvesc import VESC
import time
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tamiya_jetracer.msg import vesc_cmd
from tamiya_jetracer.msg import vesc_feedback
from math import *



# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port = '/dev/vesc'

meter_per_pulse = 0.004716966
meter_per_rotate = 0.330
wheel_rpm_per_speed = 99.4213/9
ms_per_speed = (meter_per_rotate * wheel_rpm_per_speed) / 60
speed_per_ms = 1 / ms_per_speed
wheelbase = 0.320

x = vx = y = vy = th = vth = steer_rad = 0.0
currnet_tacho = last_tacho = tacho_err = 0
tacho_jitter_threshold = 10
last_time = currnet_time = 0.0
steer_offset = -0
rev_steer_offset = 0

speed_cmd = 0
steer_cmd = 0 + steer_offset

manualMode = 1


def joyCallback(data):
    global manualMode
    if data.axes[3] != 1:
        manualMode = 1
    else:
        manualMode = 0

def joy_cmdCallback(data):
    global speed_cmd, steer_cmd, steer_rad
    if manualMode != 1:
        return
    if 0.1 < data.linear.x < 0.6:
        data.linear.x = 0.6
    speed_cmd = data.linear.x * speed_per_ms
    if data.linear.x != 0:
        steer_rad = atan(wheelbase / data.linear.x * data.angular.z)
    else:
        steer_rad = 0

    if steer_rad >  0.45:
        steer_rad = 0.45
    if steer_rad <  -0.45:
        steer_rad = -0.45
    msg.steer.data = steer_rad
    steer_cmd = degrees(steer_rad)




def cmd_velCallback(data):
    global speed_cmd, steer_cmd, steer_rad
    if manualMode == 1:
        return
    if 0.1 < data.linear.x < 0.7:
        data.linear.x = 0.7
    speed_cmd = data.linear.x * speed_per_ms
    if data.linear.x != 0:
        steer_rad = atan(wheelbase / data.linear.x * data.angular.z)
    else:
        steer_rad = 0

    if steer_rad >  0.45:
        steer_rad = 0.45
    if steer_rad <  -0.45:
        steer_rad = -0.45
    msg.steer.data = steer_rad
    steer_cmd = degrees(steer_rad)
    



def set_and_get_speed(speed, steer, count):
    global last_tacho
    while True:
        try:
            if count == 0:
                motor.set_rpm(0)
                quit()
            # rospy.sleep(0.01)
            if speed < 0:
                motor.set_servo((50 - steer*1.8 - rev_steer_offset) / 100)
            else:    
                motor.set_servo((50 - steer*1.8 + steer_offset) / 100)
            
            rospy.sleep(0.02)
            motor.set_rpm(int(speed*100))
            # motor.set_rpm(1000)
            # motor.set_duty_cycle(speed/100)
            rospy.sleep(0.02)

            tacho = int(motor.get_measurements().tachometer)
            D_tacho = tacho - last_tacho
            last_tacho = tacho
            if abs(D_tacho) > tacho_jitter_threshold:
                tacho_err += D_tacho
                tacho -= tacho_err
            
            if not (tacho == 0 or None):
                break

            count -= 1
            
            

        except:
            count -= 1
            if count <= 0:
                motor.set_rpm(0)
                quit()
            set_and_get_speed(speed, steer, count)

    
    return (tacho)

def publisher(tacho_pub, steer_pub):
    msg = vesc_feedback()

    msg.tacho.data = int(tacho_pub)
    msg.steer.data = int(steer_pub)
    
    pub.publish(msg)

def initialize(count):
    try:
        init_tacho = last_tacho = int(motor.get_measurements().tachometer)
    except:
        count += 1
        
        if(count > 5):
            err;
        initialize(count)
    return init_tacho

if __name__ == '__main__':

    rospy.init_node('vesc', anonymous=True)
    
    pub =  rospy.Publisher('/vesc_feedback', vesc_feedback, queue_size=1)
    msg = vesc_feedback()
    
    
    rospy.Subscriber("/cmd_vel",Twist ,cmd_velCallback)

    rospy.Subscriber("/joy_cmd",Twist ,joy_cmdCallback)
    rospy.Subscriber("/joy",Joy ,joyCallback)
    
    
    rate = rospy.Rate(20)

    motor = VESC(serial_port=serial_port)
    init_tacho = initialize(0)
    

    
    set_and_get_speed(0, 50, 10)
    rospy.sleep(0.05)
    

    while not rospy.is_shutdown():
        
        # print(time.time())
        # rospy.sleep(0.01)
        
        # current_tacho = set_and_get_speed(8, steer_cmd, 10)
        
        msg.tacho.data = set_and_get_speed(speed_cmd, steer_cmd, 10) - init_tacho
        rospy.loginfo(msg.tacho.data)
        # rospy.loginfo(steer_rad)
        # rospy.loginfo(steer_cmd)
        pub.publish(msg)
        # print(ms_per_speed * 9)
        # print(current_tacho,type(currnet_tacho))
        # print(msg.tacho.data)
        
        

        # print(ms_per_speed)
        # print(encoder)
        
        rate.sleep()
        
    
    motor.set_rpm(0)
    rospy.sleep(0.05)
    motor.stop_heartbeat()


        
        










