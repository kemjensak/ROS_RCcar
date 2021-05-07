#!/usr/bin/env python3
import rospy
from pyvesc import VESC
import time
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from tamiya_jetracer.msg import vesc_cmd


# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port = '/dev/ttyACM1'
steer_offset = 5

speed_cmd = 0
steer_cmd = 50 + steer_offset


def callback(data):
    global speed_cmd
    global steer_cmd
    speed_cmd = data.speed.data
    steer_cmd = data.steer.data

def set_and_get_speed(speed, steer, count):
    
    if count == 0:
        return "Err"
    while True:
        try:
            # motor.set_servo((steer_cmd + steer_offset) / 100)
            # rospy.sleep(0.01)
            motor.set_servo((steer_cmd + steer_offset) / 100)
            # rospy.sleep(0.01)
            motor.set_duty_cycle(speed)
            # rospy.sleep(0.01)
            tacho = motor.get_measurements().tachometer
            print("trying")
            if not (tacho == 0 or None):
                break

        except:
            count -= 1
            set_and_get_speed(speed, steer, count)


    return tacho
       



if __name__ == '__main__':

    rospy.init_node('vesc', anonymous=True)
    pub =  rospy.Publisher('/vesc_feedback', Int16, queue_size=10)
    
    rospy.Subscriber("/vesc_cmd", vesc_cmd, callback)
    
    rate = rospy.Rate(20)

    motor = VESC(serial_port=serial_port)


    while not rospy.is_shutdown():
        
        # print(speed_cmd)
        
        # rospy.sleep(0.01)
        encoder = set_and_get_speed(speed_cmd, steer_cmd, 10)
        print(encoder)
        
        rate.sleep()
        
    
    motor.set_rpm(0)
    motor.stop_heartbeat()


        
        











