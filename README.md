# neato-driver-python

This repo contains a python driver for Neato brand robot vacuums. Use this driver to control a Neato robot using the USB port.

If you want to simply program your Neato Vacuum to do something neat the `example.py` file contains a short example. The driver is documented with Python Docstrings and follows the official documentation with some modifications to make it easy to use.

This driver is used in the [neato-ros](https://github.com/brannonvann/neato) project to turn a Neato vacuum into a [ROS](http://ros.org/) robot.

## Getting Started

1. Clone this repo to the computer.
1. Install Python 3.
1. Using a usb cable, plug your Neato robot vacuum into a computer. The Neato's Mini USB port is on the outside next to a rubber plug.
1. You may have to modify the permissions on the `example.py` file to allow it to be executed. In linux use the Terminal command `chmod +x ./example.py` while in the directory. If you don't know how to do this, search the internet.
1. Update the serial port value in the `example.py` file.
1. Run the example: `python3 example.py`.

## Usage

This driver supports much more than is documented below. The `neato_driver.py` is fully documented. Here are a few things you can do:

```python

import neato_driver as robot

# Initilize connection to the neato with logging - replace with your usb port
robot.init('/dev/tty.usbmodem14401', True)

# Play a Sound
robot.PlaySound(neato.Sounds.Alert)

# Move Wheels (left mm, right mm, speed mm/s, acceleration mm/s^2 )
robot.SetMotorWheels(50, 50, 20, 20)

# Turn on Test Mode - Required for many of the commands.
robot.TestMode(True)

# Turn on Lidar
robot.SetLDSRotation(True)

# Get Lidar Data
lidar_data = robot.GetLDSScan()

# Turn off lidar
robot.SetLDSRotation(False)

# Get the button states
buttons = robot.GetButtons()

# Get the digital sensors
sensors_digital = robot.GetDigitalSensors()

# Get the analog sensors
sensors_analog = robot.GetAnalogSensors()

# Get the motor states
motors = robot.GetMotors()

# Get the help docs from your Neato
help = robot.Help()

```

## Issues

Please report any issues to https://github.com/brannonvann/neato-driver-python/issues