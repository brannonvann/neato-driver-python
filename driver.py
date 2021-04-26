#!/usr/bin/env python3
import time
from enum import Enum
import serial

# arguments
port = '/dev/tty.usbmodem14401'  # '/dev/neato'

# globals
serialPort = serial.Serial(port, timeout=1)


class LED(Enum):
    BacklightOn = "BacklightOn"
    BacklightOff = "BacklightOff"
    ButtonAmber = "ButtonAmber"
    ButtonGreen = "ButtonGreen"
    LEDRed = "LEDRed"
    LEDGreen = "LEDGreen"
    ButtonAmberDim = "ButtonAmberDim"
    ButtonGreenDim = "ButtonGreenDim"
    ButtonOff = "ButtonOff"


class Sounds(Enum):
    Stop = -1
    WakingUp = 0
    StartingCleaning = 1
    CleaningCompleted = 2
    AttentionNeeded = 3
    BackingUpIntoBaseStation = 4
    BaseStationDockingCompleted = 5
    TestSound1 = 6
    TestSound2 = 7
    TestSound3 = 8
    TestSound4 = 9
    TestSound5 = 10
    Exploring = 11
    ShutDown = 12
    PickedUp = 13
    GoingToSleep = 14
    ReturningHome = 15
    UserCanceledCleaning = 16
    UserTerminatedCleaning = 17
    SlippedOffBaseWhileCharging = 18
    Alert = 19
    ThankYou = 20


class SystemModes(Enum):
    Shutdown = "Shutdown"
    Hibernate = "Hibernate"
    Standby = "Standby"


class ScheduleTypes(Enum):
    House = "House"
    None_ = "None"


class CleanMode(Enum):
    Stop = "Stop"
    House = "House"
    Spot = "Spot"


class OnToggle(Enum):
    On = "On"
    Off = "Off"


class EnableToggle(Enum):
    On = "On"
    Off = "Off"


def log(message):
    print(message)


def debug(message):
    print(message)


def connect():
    global serialPort, port
    cmdTestMode(True)

    # if (len(response) > 0):
    #     print("Connected to Neato on port" + port)
    #     return True
    # else:
    #     print("not connected to neato")
    #     return False


def write(message):
    global serialPort
    serialPort.flush()
    debug(message)
    serialPort.write(str.encode(message + '\n'))
    serialPort.readline()  # clear sent command
    return serialPort.readlines()


def parseResponse(lines, intKeys=None, floatKeys=None, boolKeys=None):
    # Parses response from neato and returns formatted result
    results = dict()

    for line in lines[1:-1]:

        (key, val) = line.decode('ascii').split(",")

        if intKeys and key in intKeys:
            results[key] = int(val)
        elif floatKeys and key in floatKeys:
            results[key] = float(val)
        elif boolKeys and key in boolKeys:
            results[key] = bool(val)
        else:
            results[key] = val.strip()
    return results


def cmdClean(mode: CleanMode = CleanMode.Stop):
    """Neato API Command: Clean.
    Starts a cleaning by simulating press of start button.

    Args:
        mode (CleanMode, optional): The Clean mode to set. One of "Stop", "Spot", or "House". Defaults to CleanMode.Stop.
    """

    write("Clean " + mode.value)


def cmdDiagTest(testsOff=False,
                drivePath=False,
                driveForever=False,
                moveAndBump=False,
                dropTest=False,
                autoCycle=False,
                oneShot=False,
                brushOn=False,
                vacuumOn=False,
                ldsOn=False,
                allMotorsOn=False,
                disablePickupDetect=False,
                drivePathDist=None,
                driveForeverLeftDist=None,
                driveForeverRightDist=None,
                driveForeverSpeed=None,
                speed=None,
                brushSpeed=None):
    # Neato API Command: DiagTest.
    # Executes different test modes. Once set, press Start button to engage. (Test modes are mutually exclusive.)

    results = write(
        "DiagTest" + (" TestsOff" if testsOff else "") +
        (" DrivePath" if drivePath else "") +
        (" DriveForever" if driveForever else "") +
        (" MoveAndBump" if moveAndBump else "") +
        (" DropTest" if dropTest else "") +
        (" AutoCycle" if autoCycle else "") + (" OneShot" if oneShot else "") +
        (" BrushOn" if brushOn else "") + (" VacuumOn" if vacuumOn else "") +
        (" LDSOn" if ldsOn else "") + (" AllMotorsOn" if allMotorsOn else "") +
        (" DisablePickupDetect" if disablePickupDetect else "") +
        (" " + str(drivePathDist) if drivePathDist else "") +
        (" " + str(driveForeverLeftDist) if driveForeverLeftDist else "") +
        (" " + str(driveForeverRightDist) if driveForeverRightDist else "") +
        (" " + str(driveForeverSpeed) if driveForeverSpeed else "") +
        (" " + str(speed) if speed else "") +
        (" " + str(brushSpeed) if brushSpeed else ""))
    return parseResponse(results)


def cmdGetAccel():
    """Neato API Command: GetAccel.
    Get the Accelerometer readings.

    Returns:
        [type]: [description]
    """

    results = write("GetAccel")
    return parseResponse(results, floatKeys=["RollInDegrees", "PitchInDegrees", "XInG", "YInG", "ZInG", "SumInG"])


def cmdGetAnalogSensors(raw=False, stats=False):
    # Neato API Command: GetAnalogSensors.
    # Get the A2D readings for the analog sensors.

    results = write("GetAnalogSensors" + (" raw" if raw else "") +
                    (" stats" if stats else ""))
    return parseResponse(results)


def cmdGetButtons():
    # Neato API Command: GetButtons.
    # Get the state of the UI Buttons.

    results = write("GetButtons")
    return parseResponse(results)


def cmdGetCalInfo():
    # Neato API Command: GetCalInfo.
    # Prints out the cal info from the System Control Block.

    results = write("GetCalInfo")
    return parseResponse(results)


def cmdGetCharger():
    # Neato API Command: GetCharger.
    # Get the diagnostic data for the charging system.

    results = write("GetCharger")
    return parseResponse(results)


def cmdGetDigitalSensors():
    # Neato API Command: GetDigitalSensors.
    # Get the state of the digital sensors

    results = write("GetDigitalSensors")
    return parseResponse(results)


def cmdGetErr(clear=False):
    # Neato API Command: GetErr.
    # Get Error Message.

    results = write("GetErr" + (" Clear" if clear else ""))
    return parseResponse(results)


def cmdGetLDSScan():
    # Neato API Command: GetLDSScan.
    # Get scan packet from LDS.

    results = write("GetLDSScan")
    return parseResponse(results)


def cmdGetLifeStatLog():
    # Neato API Command: GetLifeStatLog.
    # Get All Life Stat Logs.

    results = write("GetLifeStatLog")
    return parseResponse(results)


def cmdGetMotors(brush=False,
                 vacuum=False,
                 leftWheel=False,
                 rightWheel=False,
                 laser=False,
                 charger=False):
    # Neato API Command: GetMotors.
    # Get the diagnostic data for the motors.

    results = write("GetMotors" + (" Brush" if brush else "") +
                    (" Vacuum" if vacuum else "") +
                    (" LeftWheel" if leftWheel else "") +
                    (" RightWheel" if rightWheel else "") +
                    (" Laser" if laser else "") +
                    (" Charger" if charger else ""))
    return parseResponse(results)


def cmdGetSchedule(day=None):
    # Neato API Command: GetSchedule.
    # Get the Cleaning Schedule. (24 hour clock format)

    results = write("GetSchedule" + (" " + str(day) if day else ""))
    return parseResponse(results)


def cmdGetSysLog():
    # Neato API Command: GetSysLog.
    # Get System Log data.

    results = write("GetSysLog")
    return parseResponse(results)


def cmdGetTime():
    # Neato API Command: GetTime.
    # Get Current Scheduler Time.

    results = write("GetTime")
    return parseResponse(results)


def cmdGetVersion():
    # Neato API Command: GetVersion.
    # Get the version information for the system software and hardware.

    results = write("GetVersion")
    return parseResponse(results)


def cmdGetWarranty():
    # Neato API Command: GetWarranty.
    # Get the warranty validation codes.

    results = write("GetWarranty")
    return parseResponse(results)


def cmdHelp(cmd=None):
    # Neato API Command: Help.
    # Without any argument, this prints a list of all possible cmds. With a command name, it prints the help for that particular command

    results = write("GetErr" + (" " + str(cmd) if cmd else ""))
    return parseResponse(results)


def cmdPlaySound(sound: Sounds = Sounds.Stop):
    """Neato API Command: PlaySound.
        Play the specified sound in the robot.

    Args:
        sound (Sounds, optional): Specify the sound to play. Defaults to Sounds.Stop.
        Sounds:
        0 – Waking Up
        1 – Starting Cleaning
        2 – Cleaning Completed
        3 – Attention Needed
        4 – Backing up into base station
        5 – Base Station Docking Completed
        6 – Test Sound 1
        7 – Test Sound 2
        8 – Test Sound 3
        9 – Test Sound 4
        10 – Test Sound 5
        11 – Exploring
        12 – ShutDown
        13 – Picked Up
        14 – Going to sleep
        15 – Returning Home
        16 – User Canceled Cleaning
        17 – User Terminated Cleaning
        18 – Slipped Off Base While Charging
        19 – Alert
        20 – Thank You
    """

    write("PlaySound " + (str(sound) if not sound == Sounds.Stop else "") +
          ("Stop" if sound == Sounds.Stop else ""))


def cmdRestoreDefaults():
    # Neato API Command: RestoreDefaults.
    # Restore user settings to default

    results = write("RestoreDefaults")
    return parseResponse(results)


def cmdSetDistanceCal():
    # Neato API Command: SetDistanceCal.
    # Set distance sensor calibration values for min and max distances.

    results = write("SetDistanceCal")
    return parseResponse(results)


def cmdSetFuelGauge():
    # Neato API Command: SetFuelGauge.
    # Set Fuel Gauge Level.

    results = write("SetFuelGauge")
    return parseResponse(results)


def cmdSetLCD():
    # Neato API Command: SetLCD.
    # Sets the LCD to the specified display. (TestMode Only)

    results = write("SetLCD")
    return parseResponse(results)


def cmdSetLDSRotation():
    # Neato API Command: SetLDSRotation.
    # Sets LDS rotation on or off. Can only be run in TestMode.

    results = write("SetLDSRotation")
    return parseResponse(results)


def cmdSetLED():
    # Neato API Command: SetLED.
    # Sets the specified LED to on,off,blink, or dim. (TestMode Only)

    results = write("SetLED")
    return parseResponse(results)


def cmdSetMotorBrush(rpm, brushEnable: EnableToggle):
    # Neato API Command: SetMotor. Seperated cmdSetMotor for ease of use.
    # Sets the specified motor to run in a direction at a requested speed. (TestMode Only)

    results = write("SetMotor brush" +
                    (" BrushEnable" if brushEnable else " BrushDisable"))
    return parseResponse(results)


def cmdSetMotorVacuum(rpm=0, percent=0):
    # Neato API Command: SetMotor. Seperated cmdSetMotor for ease of use.
    # Sets the specified motor to run in a direction at a requested speed. (TestMode Only)

    vacuumOnOff = "VacuumOff"

    if (rpm and rpm > 0) or (percent and percent > 0):
        vacuumOnOff = "VacuumOn"

    write("SetMotor " + (" RPM " + str(rpm) if rpm and rpm > 0 else "") +
          (" VacuumSpeed " + str(percent) if percent and percent > 0 else ""))
    return


def cmdSetMotorWheelsEnable(rWheel=True, lWheel=True):

    write("SetMotor" + (" RWheelEnable" if rWheel else " RWheelDisable")
          + (" LWheelEnable" if lWheel else " LWheelDisable")
          )


def cmdSetMotorWheels(lWheelDist=0, rWheelDist=0, speed=1, accel=None):
    # Neato API Command: SetMotor. Seperated cmdSetMotor for ease of use.
    # Sets the specified motor to run in a direction at a requested speed. (TestMode Only)

    write("SetMotor" + " lWheelDist " + str(lWheelDist) +
          " rWheelDist " + str(rWheelDist) + " speed " + str(speed)
          + (" accel " + str(accel) if accel else ""))


def cmdSetSchedule(day=0,
                   hour=0,
                   minute=0,
                   scheduleType: ScheduleTypes = ScheduleTypes.House,
                   enabled: OnToggle = OnToggle.On):
    """Neato API Command: SetSchedule.
    Modify Cleaning Schedule

    Args:
        day (int, optional): Day of week value Sunday=0,Monday=1. Defaults to 0.
        hour (int, optional): Hour value 0..23. Defaults to 0.
        minute (int, optional): Minutes value 0..59. Defaults to 0.
        scheduleType (ScheduleTypes, optional): The Schedule type to set. Defaults to ScheduleTypes.House.
        enabled (OnToggle, optional): Enable or Disable Scheduled cleanings. Defaults to OnToggle.On.

    Returns:
        [type]: [description]
    """

    results = write("SetSchedule" + (" " + str(day) if day else "") +
                    (" " + str(hour) if hour else "") +
                    (" " + str(minute) if minute else "") +
                    (" " + scheduleType) + (" " + enabled))
    return parseResponse(results)


def cmdSetSystemMode(mode: SystemModes):
    """Neato API Command: SetSystemMode.
    Set the operation mode of the robot. (TestMode Only)

    Args:
        mode (SystemModes): The system mode to set.

    Returns:
        [type]: [description]
    """

    results = write("SetSystemMode " + mode)
    return parseResponse(results)


def cmdSetTime(day=0, hour=0, minute=0, second=0):
    """Neato API Command: SetTime.
    Sets the current day, hour, and minute for the scheduler clock.

    Args:
        day (int, optional): Day of week value Sunday=0,Monday=1. Defaults to 0.
        hour (int, optional): Hour value 0..23. Defaults to 0.
        minute (int, optional): Minutes value 0..59. Defaults to 0.
        second (int, optional): Seconds value 0..59. Defaults to 0.

    Returns:
        [type]: [description]
    """

    results = write("SetTime" + (" " + str(day) if day else "") +
                    (" " + str(hour) if hour else "") +
                    (" " + str(minute) if minute else "") +
                    (" " + str(second) if second else ""))
    return parseResponse(results)


def cmdSetWallFollower(enable):
    """Neato API Command: SetWallFollower.
        Enables/Disables wall follower

    Args:
        enable (bool): Enable or disable wall follower

    Returns:
        [type]: [description]
    """

    results = write("SetWallFollower" + (" Enable" if enable else " Disable"))
    return parseResponse(results)


def cmdTestMode(on):
    """Neato API Command: TestMode.
    Sets TestMode on or off. Some commands can only be run in TestMode.

    Args:
        on (bool): [Turns Testmode on or off.]
    """
    write("TestMode " + ("On" if on else "Off"))


def test():
    # cmdTestMode(False)
    # cmdClean(CleanMode.House)
    # time.sleep(15)
    # cmdClean(CleanMode.Spot)
    # time.sleep(15)
    # cmdClean(CleanMode.Stop)

    print(cmdGetAccel())

    # i = 0
    # cmdPlaySound()
    # while i <= 20:
    #     cmdPlaySound(i)
    #     time.sleep(1)
    #     i += 1

    # cmdSetMotorWheels(100, 100, 50)
    # cmdSetMotorWheels(100, 100, 100, 5)

    # cmdSetMotorVacuum(60)

    # cmdSetMotorVacuum(None, 90)

    # cmdSetMotorVacuum(None, 0)

    # cmdSetMotorVacuum()


def start():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(publish_frequency)  # 10hz

    connect()
    test()


if True:
    try:
        print("starting")
        start()
    except:
        raise
