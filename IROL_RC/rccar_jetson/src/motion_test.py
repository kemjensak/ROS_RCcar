#!/usr/bin/env python3
import rospy, geometry_msgs.msg, nav_msgs.msg
from jetracer.nvidia_racecar import NvidiaRacecar
import time
car = NvidiaRacecar()

car.steering = 0.0

print(car.steering_gain)
print(car.steering_offset)

car.throttle = -0.2

print(car.throttle_gain)

car.throttle_gain = 0.5

time.sleep(5)




