#!/usr/bin/env python3
import rospy
from pyvesc import VESC
import time
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import geometry_msgs.msg 
from tamiya_jetracer.msg import vesc_cmd
from math import *



# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port = '/dev/ttyACM0'

meter_per_pulse = 0.004166666
meter_per_rotate = 0.205
wheel_rpm_per_speed = 116.144/9
ms_per_speed = (meter_per_rotate * wheel_rpm_per_speed) / 60
speed_per_ms = 1 / ms_per_speed
wheelbase = 0.257

x = vx = y = vy = th = vth = steer_rad = 0.0
currnet_tacho = last_tacho =0
last_time = currnet_time = 0.0
steer_offset = 0

speed_cmd = 0
steer_cmd = 50 + steer_offset



def callback(data):
    global speed_cmd, steer_cmd, steer_rad
    speed_cmd = data.linear.x
    steer_rad = atan(wheelbase / data.linear.x * data.angular.z)
    steer_cmd = degrees(steer_rad)

def cal_odom(delta_tacho, vth): #vth = steer_rad
    global x, y, th
    if last_time == 0 or :
        return
    else:
        current_time = time.time()
        dt = current_time - last_time

        v = (delta_tacho * meter_per_pulse) / dt
        # vth = (v / wheelbase) * tan(steer)
        vx = v * cos(th)
        vy = v * sin(th)

        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th
    last_time = time.time()



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


if __name__ == '__main__':

    rospy.init_node('vesc', anonymous=True)
    
    pub =  rospy.Publisher('/vesc_feedback', geometry_msgs.msg.Twist, queue_size=10)
    
    rospy.Subscriber("/cmd_vel",geometry_msgs.msg.Twist ,callback)
    
    
    rate = rospy.Rate(20)

    motor = VESC(serial_port=serial_port)


    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        # print(speed_cmd)
        print(time.time())
        # rospy.sleep(0.01)
        
        current_tacho = set_and_get_speed(0, steer_cmd, 10)
        if last_tacho != 0
        delta_tacho = current_tacho - last_tacho
        cal_odom(delta_tacho, vth)
        last_tacho
        # print(ms_per_speed)
        # print(encoder)
        
        rate.sleep()
        
    
    motor.set_rpm(0)
    motor.stop_heartbeat()


        
        











