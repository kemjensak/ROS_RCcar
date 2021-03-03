from jetracer.nvidia_racecar import NvidiaRacecar
import time
car = NvidiaRacecar()

car.steering = 0.9

print(car.steering_gain)
print(car.steering_offset)

car.throttle = 0.5

print(car.throttle_gain)

car.throttle_gain = 0.5

time.sleep(5)




