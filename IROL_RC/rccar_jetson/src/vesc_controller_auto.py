#!/usr/bin/env python3
import rospy
from pyvesc import VESC
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rccar_jetson.msg import vesc_feedback
from autoware_msgs.msg import VehicleCmd
from math import *

meter_per_rotate = 0.330
wheel_rpm_per_speed = 99.4213/9
ms_per_speed = (meter_per_rotate * wheel_rpm_per_speed) / 60
speed_per_ms = 1 / ms_per_speed * 1.3
wheelbase = 0.320

class RC_driver():
    def __init__(self):
        self.errCount = 0
        self.max_errCount = 5
        self.serial_port = rospy.get_param('~serial_port','/dev/vesc')
        self.steer_offset = rospy.get_param('~steer_offset', -4.5)     
        self.rev_steer_offset = rospy.get_param('~rev_steer_offset',3.8)   
        self.wheelbase = rospy.get_param('~wheelbase',0.320)
        self.maxSteer = rospy.get_param('~maxSteer',0.45) # rad
        self.tacho_jitter_threshold = rospy.get_param('~tacho_jitter_threshold',60)
        self.connectVESC()
        self.getInitialTacho()
        self.tacho_err = 0
        self.autoMode = 0
        self.lastMode = 0
        self.steer_rad = 0.0
        self.speed_cmd = 0
        self.steer_cmd = 0
        self.init_tacho = self.set_and_get_vesc(0, 50)

        self.feedback_msg = vesc_feedback()
        self.vehiclecmd = VehicleCmd()

        self.feedbackPublisher =  rospy.Publisher('/vesc_feedback', vesc_feedback, queue_size=1)
        
        # rospy.Subscriber("/cmd_vel",Twist ,self.cmd_velCallback) # auto
        rospy.Subscriber('/vehicle_cmd', VehicleCmd, self.vehiclecmdCallback)

        rospy.Subscriber("/joy_cmd",Twist ,self.joy_cmdCallback)
        rospy.Subscriber("/joy",Joy ,self.joyCallback)
    
        
        rospy.sleep(0.05)

    def connectVESC(self):
        try:
            self.motor = VESC(serial_port=self.serial_port)
        except Exception as e:
            rospy.logwarn("Opening serial failed during init! reconnecting %d times" % (self.errCount + 1))
            rospy.sleep(3)
            self.handleException(e)
            self.motor = self.connectVESC()
        self.errCount = 0

    def getInitialTacho(self):
        try:
            self.init_tacho = self.last_tacho = int(self.motor.get_measurements().tachometer)
        except Exception as e:
            rospy.logwarn("Getting initail tacho failed during init! reconnecting %d times" % (self.errCount + 1))
            rospy.sleep(1)
            self.handleException(e)
            self.getInitialTacho()
        self.errCount = 0

    def joyCallback(self, data):
        if data.axes[3] == -1.0:
            if self.lastMode == 0:
                self.speed_cmd = 0
                self.steer_cmd = 0
            self.autoMode = 1
            self.lastMode = 1
        else:
            if self.lastMode == 1:
                self.speed_cmd = 0
                self.steer_cmd = 0
            self.autoMode = 0
            self.lastMode = 0

    def vehiclecmdCallback(self, data):
        if self.autoMode == 0:
            return
        lin_cmd = data.ctrl_cmd.linear_velocity
        ang_cmd = data.ctrl_cmd.steering_angle
        self.cal_cmd(lin_cmd, 0)
        self.steer_cmd = degrees(ang_cmd)

    def joy_cmdCallback(self, data):
        self.manualCmdLinear = data.linear.x
        self.manualCmdAngular = data.angular.z
        if self.autoMode == 1:
            return
        self.cal_cmd(self.manualCmdLinear, self.manualCmdAngular)
    
    def cmd_velCallback(self, data):
        if self.autoMode == 0:
            return
        self.cal_cmd(data.linear.x, data.angular.z)
        
    def cal_cmd(self, lin_spd, ang_spd):
        # if 0.1 < lin_spd < 0.7:
        #     lin_spd = 0.7
        self.speed_cmd = lin_spd * speed_per_ms
        
        if lin_spd != 0:
            self.steer_rad = atan(wheelbase / lin_spd * ang_spd)
        else:
            self.steer_rad = 0
        if self.steer_rad >  self.maxSteer:
            self.steer_rad = self.maxSteer
        if self.steer_rad <  -self.maxSteer:
            self.steer_rad = -self.maxSteer
        self.feedback_msg.steer.data = self.steer_rad
        self.steer_cmd = degrees(self.steer_rad)

    def set_and_get_vesc(self, speed, steer):
        try:
            if speed < 0:
                self.motor.set_servo((50 - steer*1.8 - self.rev_steer_offset) / 100) # 1.8 -> servo/steer(deg)
                # self.motor.set_servo((((self.rev_steer_offset - 27.6) / 30) * steer + (50 - self.rev_steer_offset)) / 100) 잘못됨
            else:
                self.motor.set_servo((50 - steer*1.8 + self.steer_offset) / 100)
                # self.motor.set_servo((((self.steer_offset - 27.6) / 30) * steer + (50 - self.rev_steer_offset)) / 100) 잘못됨
            
            rospy.sleep(0.02)
            self.motor.set_rpm(int(speed*100))
            # motor.set_rpm(1000)
            # motor.set_duty_cycle(speed/100)
            rospy.sleep(0.02)
            measurements = self.motor.get_measurements()
            
            tacho = int(measurements.tachometer)
            D_tacho = tacho - self.last_tacho
            self.last_tacho = tacho
            if abs(D_tacho) > self.tacho_jitter_threshold:
                self.tacho_err += D_tacho
            tacho -= self.tacho_err
            
            
        except Exception as e:
            self.handleException(e)
            tacho = self.set_and_get_vesc(speed, steer)

        self.errCount = 0
        return tacho

    def handleException(self,e):
        rospy.logwarn(e)
        self.errCount += 1
        if self.errCount >= self.max_errCount:
            rospy.logerr("VESC driver error!")
            rospy.logerr(e)
            if self.motor.serial_port.is_open:
                self.motor.set_rpm(0)
                rospy.sleep(0.1)
                self.motor.set_servo(0.5)
                rospy.sleep(0.1)
                self.motor.stop_heartbeat()
                rospy.sleep(0.1)
                self.motor.serial_port.flush()
                rospy.sleep(0.1)
                self.motor.serial_port.close()
            err;


if __name__ == '__main__':
    rospy.init_node('RC_vesc_driver', anonymous=True)
    driver = RC_driver()
    rate = rospy.Rate(20)
        
    while not rospy.is_shutdown():
        driver.feedback_msg.tacho.data = driver.set_and_get_vesc(driver.speed_cmd, driver.steer_cmd) - driver.init_tacho
        # rospy.loginfo(driver.feedback_msg.tacho.data)
        rospy.loginfo("speed: %f",driver.speed_cmd)
        driver.feedbackPublisher.publish(driver.feedback_msg)
        rate.sleep()
    
    driver.motor.set_rpm(0)
    rospy.sleep(0.1)
    driver.motor.set_servo(0.5)
    rospy.sleep(0.1)
    driver.motor.stop_heartbeat()
    if driver.motor.serial_port.is_open:
        driver.motor.serial_port.flush()
        driver.motor.serial_port.close()