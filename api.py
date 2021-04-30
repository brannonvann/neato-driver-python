# Script used to read all help text from Neato.
# Simply connect Neato, update your port
# name ('/dev/neato') and run this script.
# All help markdown is written to a file in the
# same directory called neato_help.md

# Author: Brannon Vann brannon.vann@gmail.com
# License: MIT

# Run this script: python api.py

# Note: This script does not save your serial numbers. #Prevent Serial Write parts below prevent the write out serial numbers.

import neato_driver as robot

robot.init('/dev/tty.usbmodem14601')
commands = []
toc = "\n## Table of Contents\n"


def add_section(level, title):
    global toc
    toc += "\n" + ((len(level)-2) * "    ") + \
        "- [" + title + "](#" + "-".join(title.lower().split()) + ")"

    return "\n\n" + level + " " + title


def print_lines(text=None):

    markdown = "\n\n```"

    if text:
        lines = text.split('\n')
        for line in lines:
            # all lines line.find('Serial') != -1 or

            if line == '' or line.startswith("Help Strlen"):
                # ignore serial numbers and blank lines
                markdown = markdown  # do nothing
            else:
                markdown += "\n" + line

    return markdown + "\n```"


# print help output
help = robot.Help()
main = ""

for line in help.split('\n')[1:]:
    if line.find(' - ') != -1:
        parts = line.split(" - ")
        commands.append(parts[0])

# iterate help output to request command specific output
for command in commands:
    # command

    if command == "SetMotor":
        main += add_section(
            "##", "SetMotorBrush, SetMotorVacuum, SetMotorWheelsEnable, and SetMotorWheels")
    else:
        main += add_section("##", command)
    # command help

    desc = "\n\nThe below description from the Neato vacuum maps to the `neato_driver." + \
        command + "()` function"

    if command == "SetMotor":
        desc = "\n\nThe below description from the Neato vacuum maps to the `neato_driver.`SetMotorBrush()`, `neato_driver.SetMotorVacuum()`, `neato_driver.SetMotorWheelsEnable()`, and `neato_driver.SetMotorWheels()` functions. These were divided to make it easier to integrate to the Neato."

    if command == "SetIEC":
        desc = "\n\nThe SetIEC function is not supported by this driver."

    if command == "GetSysLog":
        desc = "\n\nThe GetSysLog function was not implemented in the test Neato. The raw results are returned."

    if command == "Upload":
        desc = "\n\nThe Upload function is not supported by this driver."

    main += desc

    main += print_lines(robot.Help(command))

    # command example
    if command.startswith('Get') or command.startswith('Help'):
        fn = getattr(robot, command)
        result = fn()
        if type(result) is dict:
            for key in result:
                if str(key).find("Serial") > -1:
                    result[key] = "SERIAL-EXCLUDED"

        example = str(result)
        main += "\n\nReturns: " + "`" + str(type(result)) + "`"
        main += "\n\n**Data Example:**"
        main += print_lines(example)


header = "# API\n"
header += '\n'
header += "This describes the `neato_driver.py` API. The documentation is ordered and grouped according to the Neato API which matches the `neato_driver.py` API in all but a few cases. Any differences are noted.\n\n"
header += "Each of the `neato_driver` functions are described below along with the Neato vacuum supplied help description that describes the function and if the function returns data, the data type returned and an example is provided.\n\n"

header += "This was generated using the `api.py` script. To produce the documentation, adjust the serial port to match your Neato's and run the script.\n"

# write out file, overwrites any existing file
helpResponseProcessor = open("api.md", "w")
helpResponseProcessor.write(header+main)  # +toc
helpResponseProcessor.close()

print("Done creating api.md document")
