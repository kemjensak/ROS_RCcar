from pyvesc import VESC
import time

# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port = '/dev/ttyACM0'


# a function to show how to use the class with a with-statement
def run_motor_using_with():
    with VESC(serial_port=serial_port) as motor:
        # print("Firmware: ", motor.get_firmware_version())

        
        start_tacho = motor.get_measurements().tachometer
        print("hallsensor:",start_tacho)
        
        
        motor.set_duty_cycle(.02)
        start = time.time()

        # print("Getting values takes ", stop-start, "seconds.")
        # run motor and print out rpm for ~2 seconds
        for i in range(20):
            #time.sleep(0.05)
            # print("rpm:",motor.get_measurements().rpm)
            response_tacho = motor.get_measurements().tachometer
            print("hallsensor:",response_tacho)
            
            stop = time.time()
            if start_tacho != response_tacho:
                print("response time:",stop-start)
            time.sleep(0.02)

        motor.set_rpm(0)
        motor.set_duty_cycle(-.02)
        print("rpm:",motor.get_measurements().rpm)
        print("hallsensor:",motor.get_measurements().tachometer)
        time.sleep(2)
        motor.set_rpm(-500)
        time.sleep(2)
        print("rpm:",motor.get_measurements().rpm)
        motor.set_rpm(500)
        time.sleep(2)
        motor.set_rpm(0)
        # while True:
        #     try:
        #         time.sleep(0.5)
        #         print("hallsensor:",motor.get_measurements().tachometer)
        #     except:
        #         pass


# a function to show how to use the class as a static object.
def run_motor_as_object():
    motor = VESC(serial_port=serial_port)
    # print("Firmware: ", motor.get_firmware_version())

    # sweep servo through full range
    for i in range(100):
        time.sleep(0.01)
        motor.set_servo(i/100)

    # IMPORTANT: YOU MUST STOP THE HEARTBEAT IF IT IS RUNNING BEFORE IT GOES OUT OF SCOPE. Otherwise, it will not
    #            clean-up properly.
    motor.stop_heartbeat()


def time_get_values():
    with VESC(serial_port=serial_port) as motor:
        start = time.time()
        motor.get_measurements()
        stop = time.time()
        print("Getting values takes ", stop-start, "seconds.")


if __name__ == '__main__':
    run_motor_using_with()
    run_motor_as_object()
    #time_get_values()

