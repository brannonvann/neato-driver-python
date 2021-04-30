import time
import neato_driver as robot
from math import pi

WHEEL_TRACK = 254  # mm (center of left tire to center of right tire)

if True:
    try:
        # Update with your serial port name here
        robot.init('/dev/tty.usbmodem14401', True)

        # Turn on Test Mode - Required for many of the commands
        robot.TestMode(True)

        # set neato's schedule clock to now.
        robot.SetTime()

        turn360 = WHEEL_TRACK*pi
        turn270 = turn360*3/4
        turn180 = turn360*1/2
        turn90 = turn360*1/4

        # Forward 100mm
        robot.SetMotorWheels(100, 100, 100, 20)
        time.sleep(5)

        # 360 Circle
        robot.SetMotorWheels(turn360, -turn360, 100, 20)
        time.sleep(5)

        # Box
        i = 0
        while i < 4:
            robot.SetMotorWheels(500, 500, 100, 20)
            time.sleep(10)
            robot.SetMotorWheels(turn90, -turn90, 100, 20)
            time.sleep(5)
            i += 1

    except:
        raise
