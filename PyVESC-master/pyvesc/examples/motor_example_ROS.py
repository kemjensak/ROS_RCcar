#!/usr/bin/env python3
import rospy
from pyvesc import VESC
import time

# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port = '/dev/ttyACM0'
vesc_cmd = 0

def callback(data):
    vesc_cmd = data.data


# a function to show how to use the class with a with-statement
def get_encoder_value(count):
    with VESC(serial_port=serial_port) as motor:
    
        if count == 0:
            return "Err"
        try:
            tacho = motor.get_measurements().tachometer
            print("hallsensor:",tacho)
        except:
            count -= 1
            get_encoder_value(count)


        return tacho

def set_speed(speed, count):
    with VESC(serial_port=serial_port) as motor:
    
        if count == 0:
            return "Err"
        try:
            tacho = motor.set_duty_cycle(speed)

        except:
            count -= 1
            set_speed(speed, count)


        return
       



if __name__ == '__main__':

    rospy.init_node('vesc', anonymous=True)
    pub =  rospy.Publisher('/vesc_feedback', Int16, queue_size=10)
    
    rospy.Subscriber("vesc_cmd" Int16, callback)
    
    rate = rospy.Rate(10)

    

    while not rospy.is_shutdown():

        pub.publish(get_encoder_value(10))
        set_speed(0.2, 10)
        rate.sleep()

        











