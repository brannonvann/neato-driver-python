# Python Driver for Neato Vaccums
# Author: Brannon Vann <brannon dot vann at gmail dot com>
# Originating Source: https://github.com/brannonvann/neato-driver-python
# This software may be used for commercial or non-commercial purposes.
# Any use of this code must reference the author and originating source.

# Initialize with init() and set the serial port name
# Function names match Neato's API documentation including Help()
# Help may be used to get documentation directly from the Neato robot.
# Reference the example scripts available at the source above for guidance.

import time
import datetime
from enum import Enum
import serial

__serialPort = None
__debug = False
__timeout = 0.5  # second(s)


class BacklightStatus(Enum):
    On = "BacklightOn"
    Off = "BacklightOff"


class LCDColors(Enum):
    Black = "Black"
    White = "White"


class TestModes(Enum):
    TestsOff = "TestsOff"
    DrivePath = "DrivePath"
    DriveForever = "DriveForever"
    MoveAndBump = "MoveAndBump"
    DropTest = "DropTest"


class TestMotorOptions(Enum):
    BrushOn = "BrushOn"
    BrushAndLDSOn = "BrushOn LDSOn"
    BrushAndVacuumOn = "BrushOn VacuumOn"
    VacuumOn = "VacuumOn"
    VacuumAndLDSOn = "VacuumOn LDSOn"
    LDSOn = "LDSOn"
    AllMotorsOn = "AllMotorsOn"


class LCDBars(Enum):
    Horizontal = "H"
    Vertical = "V"


class ButtonColors(Enum):
    Off = "ButtonOff"
    Amber = "ButtonAmber"
    AmberDim = "ButtonAmberDim"
    LEDRed = "LEDRed"
    LEDGreen = "LEDGreen"
    Green = "ButtonGreen"
    GreenDim = "ButtonGreenDim"


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
    PowerCycle = "PowerCycle"


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


class Calibrations(Enum):
    Minimum = "Minimum"
    Middle = "Middle"
    Maximum = "Maximum"


def __log(message):
    if __debug:
        print(message)


def __write(message):
    __log(message)
    __serialPort.flush()
    __serialPort.write(str.encode(message + "\n"))
    buffer = list()
    start = time.time()
    while True and (time.time() - start) < __timeout:
        if __serialPort.in_waiting > 0:  # for PySerial before v3.0: inWaiting()
            lines = __serialPort.readlines()
            if lines[-1] == b"\x1a":
                for line in lines[0:-1]:
                    buffer.append(line.decode().strip())
                return buffer
            else:
                for line in lines:
                    buffer.append(line.decode().strip())

        time.sleep(0.01)


def __parse_response(
    lines,
    intKeys=None,
    floatKeys=None,
    boolKeys=None,
    allInt=False,
    allFloat=False,
    allBool=False,
):
    # Parses response from neato and returns formatted result
    results = dict()

    for line in lines[2:]:

        parts = line.split(",")

        if parts[0].isnumeric():
            # Handle Laser Data
            results[int(parts[0])] = [int(parts[1]),
                                      int(parts[2]), int(parts[3])]
            continue

        if allInt or (intKeys and parts[0] in intKeys):
            results[parts[0]] = int(parts[1])
        elif allFloat or (floatKeys and parts[0] in floatKeys):
            results[parts[0]] = float(parts[1])
        elif allBool or (boolKeys and parts[0] in boolKeys):
            results[parts[0]] = parts[1] == "1"
        else:
            results[parts[0]] = parts[1]
    return results


def Clean(mode: CleanMode = CleanMode.Stop):
    """Neato API Command: Clean.
    Starts a cleaning by simulating press of start button.

    Args:
        mode (CleanMode, optional): The Clean mode to set. One of "Stop", "Spot", or "House". Defaults to CleanMode.Stop.
    """

    __write("Clean " + mode.value)


def DiagTest(
    testMode: TestModes = TestModes.TestsOff,
    motorOption: TestMotorOptions = None,
    autoCycle=False,
    oneShot=False,
    disablePickupDetect=False,
    drivePathDist=-1,
    driveForeverLeftDist=-1,
    driveForeverRightDist=-1,
    driveForeverSpeed=-1,
    speed=-1,
    brushSpeed=-1,
):
    """Neato API Command: DiagTest.
    Executes different test modes. Once set, press Start button to engage.

    Args:
        testMode (TestModes, optional): Defaults to TestModes.TestsOff.
        - TestsOff: Stop Diagnostic Test and clears all diagnostic test modes.
        - DrivePath: Sets DrivePath TestMode. Press start button to start. Robot travels straight by commanded distance as path. Mutually exclusive with other diagtest modes. Use ‘TestsOff’ option to stop.
        - DriveForever: Sets DriveForever TestMode. Press start button to start. Robot drives continuously. Mutually exclusive with other diagtest modes. Use ‘TestsOff’ option to stop.
        - MoveAndBump: Sets Move and Bump TestMode. Press start button to start. Executes canned series of motions, but will react to bumps. Mutually exclusive with other diagtest modes.
        - DropTest: Enables DropTest. Robot drives forward until a drop is detected. Mutually exclusive with other diagtest modes.

        motorOption (TestMotorOptions, optional): Defaults to None.
        - BrushOn: Turns on brush during test. May conflict with motor commands of test so use carefully!
        - VacuumOn: Turns on vacuum during test. May conflict with motor commands of test so use carefully!
        - LDSOn: Turns on LDS during test. May conflict with motor commands of test so use carefully!
        - AllMotorsOn: Turns on brush, vacuum, and lds during test. May conflict with motor commands of test so use carefully!

        autoCycle (bool, optional): DropTest argument to enable automatic restart of the test. The robot will drive backwards and then forward until a drop is detected until the test is over. Defaults to False.
        oneShot (bool, optional): Only executes test once. Defaults to False.
        disablePickupDetect (bool, optional): Ignores pickup (wheel suspension). By default, pickup detect is enabled and stops the test. Defaults to False.
        drivePathDist (int, optional): Distance in mm. Defaults to -1 (not sent).
        driveForeverLeftDist (int, optional): Use next arg to set left wheel dist for DriveForever test. Requires DriveForeverRightDist as well. The ratio of this value to DriveForeverRightDist determines turn radius. Defaults to -1 (not sent).
        driveForeverRightDist (int, optional): Use next arg to set right wheel dist for DriveForever test. Requires DriveForeverLeftDist as well. The ratio of this value to DriveForeverLeftDist determines turn radius. Defaults to -1 (not sent).
        driveForeverSpeed (int, optional): Use next arg to set turn speed of outer wheel for DriveForever test in mm/s. Defaults to -1 (not sent).
        speed (int, optional): DropTest argument to set the robot speed in mm/s. Defaults to -1 (not sent).
        brushSpeed (int, optional): DropTest argument to set the speed of the brush in rpm. Defaults to -1 (not sent).
    """
    __write(
        "DiagTest "
        + testMode.value
        + (" " + str(motorOption.value) if motorOption else "")
        + (" AutoCycle" if autoCycle else "")
        + (" OneShot" if oneShot else "")
        + (" DisablePickupDetect" if disablePickupDetect else "")
        + (" DrivePathDist " + str(drivePathDist) if drivePathDist > -1 else "")
        + (
            " DriveForeverLeftDist " + str(driveForeverLeftDist)
            if driveForeverLeftDist > -1
            else ""
        )
        + (
            " DriveForeverRightDist " + str(driveForeverRightDist)
            if driveForeverRightDist > -1
            else ""
        )
        + (
            " DriveForeverSpeed " + str(driveForeverSpeed)
            if driveForeverSpeed > -1
            else ""
        )
        + (" Speed " + str(speed) if speed > -1 else "")
        + (" BrushSpeed " + str(brushSpeed) if brushSpeed > -1 else "")
    )


def GetAccel():
    """Neato API Command: GetAccel.
    Get the Accelerometer readings.

    Returns:
         dict: Example: {'PitchInDegrees': 1.42, 'RollInDegrees': 0.97, 'XInG': 0.025, 'YInG': 0.017, 'ZInG': 1.007, 'SumInG': 1.007}
    """

    results = __write("GetAccel")
    return __parse_response(
        results,
        floatKeys=["PitchInDegrees", "RollInDegrees",
                   "XInG", "YInG", "ZInG", "SumInG"],
    )


def GetAnalogSensors(raw=False, stats=False):
    """Neato API Command: GetAnalogSensors.
    Get the A2D readings for the analog sensors.

    Args:
        raw (bool, optional): Return raw analog sensor values as milliVolts. (Default is sensor values in native units of what they measure.). Defaults to False.
        stats (bool, optional): Return stats (avg,max,min,dev,cnt) of raw analog sensor values as milliVolts.(Implies ‘raw’ option). Defaults to False.

    Returns:
         dict: Examples:

        Standard: {'WallSensorInMM': 40, 'BatteryVoltageInmV': 3792, 'LeftDropInMM': 71, 'RightDropInMM': 65, 'LeftMagSensor': -758, 'RightMagSensor': -758, 'UIButtonInmV': 3324, 'VacuumCurrentInmA': 0,
            'ChargeVoltInmV': 23985, 'BatteryTemp0InC': 28, 'BatteryTemp1InC': '26', 'CurrentInmA': '87', 'SideBrushCurrentInmA': '0', 'VoltageReferenceInmV': '1225', 'AccelXInmG': '24', 'AccelYInmG': 16, 'AccelZInmG': 1016}

        Raw: {'WallSensorInMM': 848, 'BatteryVoltageInmV': 3819, 'LeftDropInMM': 823, 'RightDropInMM': 877, 'LeftMagSensor': 0, 'RightMagSensor': 0, 'UIButtonInmV': 3321, 'VacuumCurrentInmA': 0, 'ChargeVoltInmV': 23969,
            'BatteryTemp0InC': 28, 'BatteryTemp1InC': '26', 'CurrentInmA': '119', 'SideBrushCurrentInmA': '0', 'VoltageReferenceInmV': '1224', 'AccelXInmG': '24', 'AccelYInmG': 16, 'AccelZInmG': 1008}

        Stats: {}
    """
    results = __write(
        "GetAnalogSensors" + (" raw" if raw else "") +
        (" stats" if stats else "")
    )
    return __parse_response(results, allInt=True)


def GetButtons():
    """Neato API Command: GetButtons.
    Get the state of the UI Buttons.

    Returns:
         dict: Example: {'BTN_SOFT_KEY': False, 'BTN_SCROLL_UP': False, 'BTN_START': False, 'BTN_BACK': False, 'BTN_SCROLL_DOWN': False}
    """

    results = __write("GetButtons")
    return __parse_response(results, allBool=True)


def GetCalInfo():
    """Neato API Command: GetCalInfo.
    Prints out the cal info from the System Control Block.

    Returns:
        dict: Example: {'LDSOffset': 0, 'XAccel': 0, 'YAccel': 0, 'ZAccel': 0, 'RTCOffset': 0, 'LCDContrast': 17, 'RDropMin': 293, 'RDropMid': 168, 'RDropMax': 84, 'LDropMin': 295, 'LDropMid': 168, 'LDropMax': 68, 'WallMin': 721, 'WallMid': 265, 'WallMax': 140, 'QAState': 0, 'CleaningTestSurface': 'carpet', 'CleaningTestHardSpeed': 200, 'CleaningTestCarpetSpeed': 100, 'CleaningTestHardDistance': 1200, 'CleaningTestCarpetDistance': 1200}
    """

    results = __write("GetCalInfo")
    return __parse_response(
        results,
        intKeys=[
            "LDSOffset",
            "XAccel",
            "YAccel",
            "ZAccel",
            "RTCOffset",
            "LCDContrast",
            "RDropMin",
            "RDropMid",
            "RDropMax",
            "LDropMin",
            "LDropMid",
            "LDropMax",
            "WallMin",
            "WallMid",
            "WallMax",
            "QAState",
            "CleaningTestHardSpeed",
            "CleaningTestCarpetSpeed",
            "CleaningTestHardDistance",
            "CleaningTestCarpetDistance",
        ],
    )


def GetCharger():
    """Neato API Command: GetCharger.
    Get the diagnostic data for the charging system.

    Returns:
        dict: Example: {'FuelPercent': 0, 'BatteryOverTemp': False, 'ChargingActive': False, 'ChargingEnabled': False, 'ConfidentOnFuel': False, 'OnReservedFuel': True, 'EmptyFuel': True, 'BatteryFailure': False, 'ExtPwrPresent': True, 'ThermistorPresent[0]': True, 'ThermistorPresent[1]': True, 'BattTempCAvg[0]': 27, 'BattTempCAvg[1]': 25, 'VBattV': 3.8, 'VExtV': 24.12, 'Charger_mAH': 0.0}
    """

    results = __write("GetCharger")
    return __parse_response(
        results,
        intKeys=["FuelPercent", "BattTempCAvg[0]", "BattTempCAvg[1]"],
        floatKeys=["VBattV", "VExtV", "Charger_mAH"],
        boolKeys=[
            "BatteryOverTemp",
            "ChargingActive",
            "ChargingEnabled",
            "ConfidentOnFuel",
            "OnReservedFuel",
            "EmptyFuel",
            "BatteryFailure",
            "ExtPwrPresent",
            "ThermistorPresent[0]",
            "ThermistorPresent[1]",
        ],
    )


def GetDigitalSensors():
    """Neato API Command: GetDigitalSensors.
    Get the state of the digital sensors

    Returns:
        dict: Example: {'SNSR_DC_JACK_CONNECT': False, 'SNSR_DUSTBIN_IS_IN': True, 'SNSR_LEFT_WHEEL_EXTENDED': False, 'SNSR_RIGHT_WHEEL_EXTENDED': False, 'LSIDEBIT': True, 'LFRONTBIT': True, 'RSIDEBIT': True, 'RFRONTBIT': True}
    """

    results = __write("GetDigitalSensors")
    return __parse_response(results, allBool=True)


def GetErr(clear=False):
    """Neato API Command: GetErr.
    Get Error Message.

    Args:
        clear (bool, optional): Dismiss the reported error. Defaults to False.

    Returns:
        str: Examples:

            224 - Please put my Dirt Bin back in.

            244 - Please press OKAY to restart. Battery Issue (0003)
    """

    lines = __write("GetErr" + (" Clear" if clear else ""))

    return lines[0]


def GetLDSScan():
    """Neato API Command: GetLDSScan.
    Get scan packet from LDS.

    Returns:
        dict: Example: {0: [5009, 12, 0], 1: [4991, 12, 0], 2: [5032, 11, 0], 3: [5032, 12, 0], 4: [5027, 12, 0], 5: [5037, 8, 0], 6: [5014, 10, 0], 7: [5000, 12, 0], 8: [5000, 9, 0], 9: [5078, 10, 0], 10: [5078, 9, 0], 11: [5194, 10, 0], 12: [5032, 10, 0], 13: [5107, 9, 0], 14: [0, 0, 8035], 15: [0, 0, 8035], 16: [0, 0, 8035], 17: [0, 0, 8035], 18: [0, 0, 8035], 19: [0, 0, 8035], 20: [0, 0, 8035], 21: [0, 0, 8035], 22: [0, 0, 8035], 23: [0, 0, 8035], 24: [0, 0, 8035], 25: [0, 0, 8035], 26: [0, 0, 8035], 27: [0, 0, 8035], 28: [0, 0, 8035], 29: [0, 0, 8035], 30: [0, 0, 8035], 31: [0, 0, 8035], 32: [0, 0, 8035], 33: [0, 0, 8035], 34: [0, 0, 8035], 35: [0, 0, 8035], 36: [16578, 29, 0], 37: [0, 0, 8002], 38: [0, 0, 8035], 39: [0, 0, 8035], 40: [0, 0, 8035], 41: [0, 0, 8035], 42: [16568, 77, 0], 43: [16566, 76, 0], 44: [0, 0, 8035], 45: [0, 0, 8035], 46: [0, 0, 8035], 47: [0, 0, 8035], 48: [0, 0, 8035], 49: [0, 0, 8035], 50: [0, 0, 8035], 51: [0, 0, 8035], 52: [0, 0, 8035], 53: [0, 0, 8035], 54: [16545, 38, 0], 55: [0, 0, 8035], 56: [16941, 22, 0], 57: [16544, 99, 0], 58: [0, 0, 8035], 59: [0, 0, 8035], 60: [16541, 27, 0], 61: [0, 0, 8035], 62: [16540, 34, 0], 63: [0, 0, 8035], 64: [16540, 63, 0], 65: [0, 0, 8035], 66: [0, 0, 8035], 67: [0, 0, 8035], 68: [0, 0, 8035], 69: [4991, 9, 0], 70: [4946, 12, 0], 71: [5037, 10, 0], 72: [4812, 7, 0], 73: [0, 0, 8035], 74: [0, 0, 8021], 75: [0, 0, 8035], 76: [0, 0, 8035], 77: [0, 0, 8035], 78: [0, 0, 8035], 79: [0, 0, 8035], 80: [0, 0, 8035], 81: [0, 0, 8035], 82: [0, 0, 8035], 83: [0, 0, 8035], 84: [0, 0, 8035], 85: [0, 0, 8035], 86: [0, 0, 8035], 87: [2486, 14, 0], 88: [3962, 28, 0], 89: [3801, 20, 0], 90: [0, 0, 8035], 91: [4062, 39, 0], 92: [3957, 59, 0], 93: [3923, 60, 0], 94: [3934, 57, 0], 95: [3951, 42, 0], 96: [3906, 43, 0], 97: [2639, 101, 0], 98: [3951, 30, 0], 99: [3957, 25, 0], 100: [3974, 23, 0], 101: [3960, 21, 0], 102: [3977, 13, 0], 103: [4135, 27, 0], 104: [4407, 23, 0], 105: [3553, 29, 0], 106: [2396, 53, 0], 107: [2296, 65, 0], 108: [2327, 71, 0], 109: [0, 0, 8035], 110: [0, 0, 8035], 111: [1877, 17, 0], 112: [1759, 7, 0], 113: [0, 0, 8035], 114: [0, 0, 8035], 115: [0, 0, 8035], 116: [0, 0, 8035], 117: [0, 0, 8035], 118: [0, 0, 8035], 119: [0, 0, 8035], 120: [0, 0, 8035], 121: [0, 0, 8035], 122: [0, 0, 8035], 123: [0, 0, 8035], 124: [0, 0, 8035], 125: [0, 0, 8035], 126: [0, 0, 8035], 127: [0, 0, 8035], 128: [0, 0, 8035], 129: [0, 0, 8035], 130: [0, 0, 8035], 131: [0, 0, 8035], 132: [0, 0, 8035], 133: [0, 0, 8035], 134: [0, 0, 8035], 135: [0, 0, 8035], 136: [0, 0, 8003], 137: [0, 0, 8003], 138: [0, 0, 8003], 139: [0, 0, 8035], 140: [0, 0, 8035], 141: [0, 0, 8035], 142: [0, 0, 8035], 143: [0, 0, 8035], 144: [0, 0, 8035], 145: [0, 0, 8035], 146: [0, 0, 8021], 147: [0, 0, 8035], 148: [0, 0, 8035], 149: [0, 0, 8035], 150: [0, 0, 8035], 151: [0, 0, 8035], 152: [0, 0, 8035], 153: [0, 0, 8035], 154: [0, 0, 8035], 155: [0, 0, 8035], 156: [0, 0, 8035], 157: [0, 0, 8035], 158: [0, 0, 8035], 159: [0, 0, 8035], 160: [0, 0, 8035], 161: [0, 0, 8035], 162: [0, 0, 8035], 163: [0, 0, 8035], 164: [0, 0, 8035], 165: [0, 0, 8035], 166: [0, 0, 8035], 167: [0, 0, 8035], 168: [0, 0, 8035], 169: [0, 0, 8035], 170: [0, 0, 8035], 171: [0, 0, 8035], 172: [0, 0, 8035], 173: [0, 0, 8035], 174: [0, 0, 8035], 175: [0, 0, 8035], 176: [0, 0, 8035], 177: [0, 0, 8035], 178: [0, 0, 8035], 179: [0, 0, 8035], 180: [0, 0, 8035], 181: [0, 0, 8035], 182: [0, 0, 8035], 183: [0, 0, 8035], 184: [0, 0, 8035], 185: [0, 0, 8035], 186: [0, 0, 8035], 187: [0, 0, 8035], 188: [0, 0, 8035], 189: [0, 0, 8035], 190: [0, 0, 8035], 191: [0, 0, 8035], 192: [0, 0, 8035], 193: [0, 0, 8035], 194: [0, 0, 8021], 195: [0, 0, 8035], 196: [0, 0, 8035], 197: [0, 0, 8035], 198: [0, 0, 8035], 199: [0, 0, 8035], 200: [0, 0, 8035], 201: [0, 0, 8035], 202: [0, 0, 8035], 203: [0, 0, 8035], 204: [0, 0, 8035], 205: [0, 0, 8035], 206: [0, 0, 8035], 207: [0, 0, 8035], 208: [0, 0, 8035], 209: [0, 0, 8035], 210: [0, 0, 8035], 211: [0, 0, 8035], 212: [0, 0, 8035], 213: [0, 0, 8035], 214: [0, 0, 8035], 215: [0, 0, 8035], 216: [0, 0, 8035], 217: [0, 0, 8035], 218: [0, 0, 8021], 219: [0, 0, 8035], 220: [0, 0, 8035], 221: [0, 0, 8035], 222: [0, 0, 8035], 223: [0, 0, 8035], 224: [0, 0, 8035], 225: [0, 0, 8035], 226: [0, 0, 8035], 227: [0, 0, 8035], 228: [0, 0, 8035], 229: [0, 0, 8035], 230: [0, 0, 8035], 231: [0, 0, 8035], 232: [0, 0, 8035], 233: [0, 0, 8035], 234: [0, 0, 8035], 235: [0, 0, 8035], 236: [0, 0, 8035], 237: [0, 0, 8035], 238: [0, 0, 8035], 239: [0, 0, 8035], 240: [0, 0, 8035], 241: [0, 0, 8035], 242: [0, 0, 8002], 243: [0, 0, 8035], 244: [0, 0, 8035], 245: [285, 664, 0], 246: [284, 1049, 0], 247: [286, 909, 0], 248: [286, 616, 0], 249: [16669, 116, 0], 250: [0, 0, 8035], 251: [0, 0, 8035], 252: [0, 0, 8035], 253: [0, 0, 8035], 254: [0, 0, 8035], 255: [0, 0, 8035], 256: [0, 0, 8035], 257: [16711, 72, 0], 258: [328, 439, 0], 259: [329, 831, 0], 260: [332, 902, 0], 261: [336, 898, 0], 262: [344, 836, 0], 263: [350, 849, 0], 264: [356, 784, 0], 265: [361, 767, 0], 266: [0, 767, 8021], 267: [373, 618, 0], 268: [380, 549, 0], 269: [385, 634, 0], 270: [391, 740, 0], 271: [395, 676, 0], 272: [404, 619, 0], 273: [414, 618, 0], 274: [423, 618, 0], 275: [432, 637, 0], 276: [446, 632, 0], 277: [458, 571, 0], 278: [468, 569, 0], 279: [482, 470, 0], 280: [493, 526, 0], 281: [508, 425, 0], 282: [525, 239, 0], 283: [545, 131, 0], 284: [0, 0, 8002], 285: [0, 0, 8035], 286: [0, 0, 8035], 287: [0, 0, 8035], 288: [550, 77, 0], 289: [560, 155, 0], 290: [0, 155, 8021], 291: [663, 478, 0], 292: [661, 526, 0], 293: [648, 376, 0], 294: [628, 376, 0], 295: [615, 367, 0], 296: [603, 423, 0], 297: [588, 455, 0], 298: [578, 548, 0], 299: [571, 634, 0], 300: [569, 685, 0], 301: [573, 526, 0], 302: [633, 420, 0], 303: [620, 466, 0], 304: [610, 538, 0], 305: [608, 606, 0], 306: [638, 484, 0], 307: [630, 484, 0], 308: [623, 603, 0], 309: [623, 582, 0], 310: [696, 375, 0], 311: [680, 411, 0], 312: [673, 564, 0], 313: [674, 95, 0], 314: [0, 95, 8021], 315: [0, 0, 8035], 316: [0, 0, 8002], 317: [1123, 272, 0], 318: [0, 0, 8002], 319: [1005, 380, 0], 320: [1006, 338, 0], 321: [0, 0, 8002], 322: [0, 0, 8002], 323: [976, 76, 0], 324: [0, 0, 8035], 325: [17028, 41, 0], 326: [0, 0, 8035], 327: [0, 0, 8035], 328: [0, 0, 8035], 329: [0, 0, 8035], 330: [0, 0, 8035], 331: [654, 35, 0], 332: [0, 0, 8002], 333: [659, 350, 0], 334: [1295, 37, 0], 335: [1226, 79, 0], 336: [1213, 72, 0], 337: [1213, 27, 0], 338: [0, 0, 8035], 339: [1295, 19, 0], 340: [1306, 71, 0], 341: [1129, 26, 0], 342: [0, 0, 8035], 343: [0, 0, 8035], 344: [0, 0, 8035], 345: [0, 0, 8035], 346: [0, 0, 8035], 347: [0, 0, 8035], 348: [0, 0, 8035], 349: [0, 0, 8035], 350: [0, 0, 8035], 351: [0, 0, 8035], 352: [0, 0, 8035], 353: [0, 0, 8035], 354: [0, 0, 8035], 355: [1609, 86, 0], 356: [1638, 83, 0], 357: [5088, 8, 0], 358: [5074, 10, 0], 359: [5060, 5, 0], 'ROTATION_SPEED': 5.07}
    """

    results = __write("GetLDSScan")
    return __parse_response(results, floatKeys=["ROTATION_SPEED"])


def GetLifeStatLog():
    """Neato API Command: GetLifeStatLog.
    Get All Life Stat Logs.

    Returns:
        str: Example(Sample):
        runID,statID,count,Min,Max,Sum,SumV*2
        819,LS_A2D0,37,170,184,0x000000000000198c,0x000000000011a558
        819,LS_A2D1,29997,16053,16508,0x000000001d3957b9,0x00000749df9026f5
        819,LS_A2D2,37,428,430,0x0000000000003df5,0x000000000067bf83
        819,LS_A2D3,37,415,424,0x0000000000003c7c,0x000000000062e0a2
        819,LS_A2D4,18,-1,762,0x00000000000017c9,0x000000000046cf4b
        819,LS_A2D5,18,-1,759,0x00000000000017af,0x0000000000462325
        ...
    """

    lines = __write("GetLifeStatLog")

    return "\n".join(lines)


def GetMotors(
    brush=False,
    vacuum=False,
    leftWheel=False,
    rightWheel=False,
    laser=False,
    charger=False,
):
    """Neato API Command: GetMotors.
    Get the diagnostic data for the motors.

    Args:
        brush (bool, optional): Return Brush Motor stats. Defaults to False.
        vacuum (bool, optional): Return Vacuum Motor stats. Defaults to False.
        leftWheel (bool, optional): Return LeftWheel Motor stats. Defaults to False.
        rightWheel (bool, optional): Return RightWheel Motor stats. Defaults to False.
        laser (bool, optional): Return LDS Motor stats. Defaults to False.
        charger (bool, optional): Return Battery Charger stats. Defaults to False.

    Returns:
        dict: Example:{'Brush_RPM': 0, 'Brush_mA': 0, 'Vacuum_RPM': 0, 'Vacuum_mA': 0, 'LeftWheel_RPM': 0, 'LeftWheel_Load%': 0, 'LeftWheel_PositionInMM': -1, 'LeftWheel_Speed': 0, 'RightWheel_RPM': 0, 'RightWheel_Load%': 0, 'RightWheel_PositionInMM': 0, 'RightWheel_Speed': 0, 'Charger_mAH': 0, 'SideBrush_mA': 0}
    """

    results = __write(
        "GetMotors"
        + (" Brush" if brush else "")
        + (" Vacuum" if vacuum else "")
        + (" LeftWheel" if leftWheel else "")
        + (" RightWheel" if rightWheel else "")
        + (" Laser" if laser else "")
        + (" Charger" if charger else "")
    )
    return __parse_response(results, allInt=True)


def GetSchedule(day=None):
    """Neato API Command: GetSchedule.
    Get the Cleaning Schedule. (24 hour clock format)

    Args:
        day (int, optional): Day of the week to get schedule for. Sun=0,Sat=6. If not specified, then all days are given. Defaults to None.

    Returns:
        dict: Example: {'schedule': 'disabled', 'Sun': {'start': '00:00', 'flag': 'None'}, 'Mon': {'start': '08:15', 'flag': 'H'}, 'Tue': {'start': '00:00', 'flag': 'None'}, 'Wed': {'start': '08:15', 'flag': 'H'}, 'Thu': {'start': '00:00', 'flag': 'None'}, 'Fri': {'start': '08:15', 'flag': 'H'}, 'Sat': {'start': '00:00', 'flag': 'None'}}
    """

    lines = __write("GetSchedule" + (" " + str(day)
                                     if day or day == 0 else ""))

    results = dict()

    days = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"]

    for i in range(0, len(lines)):
        line = lines[i]

        if line == "Schedule is Disabled":
            results["schedule"] = "disabled"
        elif line == "Schedule is Enabled":
            results["schedule"] = "enabled"
        elif line.split()[0] in days:
            parts = list(filter(lambda part: part != "-", line.split()))
            schedule = dict()
            schedule["start"] = parts[1]
            schedule["flag"] = parts[2]
            results[parts[0]] = schedule

    return results


def GetSysLog():
    """Neato API Command: GetSysLog.
    Get System Log data
    ***Unable to implement - Test Neato does not support this.***

    Returns:
        list: Raw Results. **Not implemented.**

    """

    return __write("GetSysLog")


def GetTime():
    """Neato API Command: GetTime.
    Get Current Scheduler Time.

    Returns:
        datetime.datetime: Examples:
                            default: 1900-01-01 00:00:00
                            set: 2021-04-22 00:05:14.571748

        Note: Day, Month, and Year are not supplied so it's assumed to be the current day or if day of week is different than today, the previous day of the week.
    """
    days = [
        "Monday",
        "Tuesday",
        "Wednesday",
        "Thursday",
        "Friday",
        "Saturday",
        "Sunday",
    ]
    line = __write("GetTime")[1]
    parts = line.split()
    timeParts = parts[1].split(":")
    dayOfWeek = days.index(parts[0])
    now = datetime.datetime.today()

    daysAdjustment = (
        0 if now.weekday() == dayOfWeek else (now.weekday() + dayOfWeek) - 7
    )

    result = now + datetime.timedelta(days=daysAdjustment)
    result = result.replace(
        hour=int(timeParts[0]), minute=int(timeParts[1]), second=int(timeParts[2])
    )

    return result


def GetVersion():
    """Neato API Command: GetVersion.
    Get the version information for the system software and hardware.

    Returns:
        dict: example: {'ModelID': ['-1', 'XV28'], 'ConfigID': ['1', ''], 'Serial Number': ['KSH17514HH', '0218400'], 'Software': ['3', '4'], 'BatteryType': ['1', 'NIMH_12CELL'], 'BlowerType': ['1', 'BLOWER_ORIG'], 'BrushSpeed': ['1200', ''], 'BrushMotorType': ['1', 'BRUSH_MOTOR_ORIG'], 'SideBrushType': ['1', 'SIDE_BRUSH_NONE'], 'WheelPodType': ['1', 'WHEEL_POD_ORIG'], 'DropSensorType': ['1', 'DROP_SENSOR_ORIG'], 'MagSensorType': ['1', 'MAG_SENSOR_ORIG'], 'WallSensorType': ['1', 'WALL_SENSOR_ORIG'], 'Locale': ['1', 'LOCALE_USA'], 'LDS Software': ['V2.6.15295', '0000000000'], 'LDS Serial': ['KSH17400AA-0263400', ''], 'LDS CPU': ['F2802x/c001', ''], 'MainBoard Vendor ID': ['505', ''], 'MainBoard Serial Number': ['200400', ''], 'BootLoader Software': ['18119', 'P'], 'MainBoard Software': ['23179', '1'], 'MainBoard Boot': ['16219', ''], 'MainBoard Version': ['4', '0'], 'ChassisRev': ['2', ''], 'UIPanelRev': ['1', '']}
    """

    lines = __write("GetVersion")[1:-1]
    results = dict()

    for line in lines:
        parts = line.split(",")
        results[parts[0]] = parts[1:3]

    return results


def GetWarranty():
    """Neato API Command: GetWarranty.
    Get the warranty validation codes.

    Returns:
        list: Example: ['000ae7ee', '0135', '38204bf6']
    """

    return __write("GetWarranty")[1:]


def Help(cmd=None):
    """Neato API Command: Help.
    Without any argument, this prints a list of all possible commands(cmd). With a command name, it prints the help for that particular command


    Args:
        cmd ([type], optional): [description]. Defaults to None.

    Returns:
        str: Example (No cmd):
            Help - Without any argument, this prints a list of all possible cmds.
            With a command name, it prints the help for that particular command
            Clean - Starts a cleaning by simulating press of start button.
            DiagTest - Executes different test modes. Once set, press Start button to engage. (Test modes are mutually exclusive.)
            GetAccel - Get the Accelerometer readings.
            GetAnalogSensors - Get the A2D readings for the analog sensors.
            GetButtons - Get the state of the UI Buttons.
            GetCalInfo - Prints out the cal info from the System Control Block.
            GetCharger - Get the diagnostic data for the charging system.
            GetDigitalSensors - Get the state of the digital sensors.
            GetErr - Get Error Message.
            GetLDSScan - Get scan packet from LDS.
            GetMotors - Get the diagnostic data for the motors.
            GetSchedule - Get the Cleaning Schedule. (24 hour clock format)
            GetTime - Get Current Scheduler Time.
            GetVersion - Get the version information for the system software and hardware.
            GetWarranty - Get the warranty validation codes.
            PlaySound - Play the specified sound in the robot.
            RestoreDefaults - Restore user settings to default.
            SetFuelGauge - Set Fuel Gauge Level.
            SetMotor - Sets the specified motor to run in a direction at a requested speed. (TestMode Only)
            SetTime - Sets the current day, hour, and minute for the scheduler clock.
            SetLED - Sets the specified LED to on,off,blink, or dim. (TestMode Only)
            SetIEC - Sets the IEC Cleaning Test parameters
            SetLCD - Sets the LCD to the specified display. (TestMode Only)
            SetLDSRotation - Sets LDS rotation on or off. Can only be run in TestMode.
            SetSchedule - Modify Cleaning Schedule.
            SetSystemMode - Set the operation mode of the robot. (TestMode Only)
            TestMode - Sets TestMode on or off. Some commands can only be run in TestMode.
            Upload - Uploads new program to the robot.
    """

    lines = __write("Help" + (" " + str(cmd) if cmd else ""))[1:-1]

    return "\n".join(lines)


def PlaySound(soundId: Sounds = Sounds.Stop):
    """Neato API Command: PlaySound.
        Play the specified sound in the robot.

    Args:
        soundId (Sounds, optional): Specify the sound to play. Defaults to Sounds.Stop.
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

    __write(
        "PlaySound "
        + ("SoundID " + str(soundId.value) if not soundId == Sounds.Stop else "")
        + ("Stop" if soundId == Sounds.Stop else "")
    )


def RestoreDefaults():
    """Neato API Command: RestoreDefaults.
    Restore user settings to default

    """

    __write("RestoreDefaults")


def SetDistanceCal(drop: Calibrations = None, wall: Calibrations = None):
    """Neato API Command: SetDistanceCal.
    Set distance sensor calibration values for min and max distances.

    Args:
        drop (Calibrations, optional): Take minimum, middle, or maximum distance drop sensor readings. Defaults to None.
        wall (Calibrations, optional): Take minimum, middle, or maximum distance wall sensor readings. Defaults to None.

    Returns:
        dict:
    """

    results = __write(
        "SetDistanceCal"
        + (" Drop" + drop.value if drop else "")
        + (" Wall" + wall.value if wall else "")
    )
    return __parse_response(results, allInt=True)


def SetFuelGauge(percent):
    """Neato API Command: SetFuelGauge.
    Set Fuel Gauge Level.

    Args:
        percent (int): Fuel Gauge percent from 0 to 100

    """

    __write("SetFuelGauge Percent " + str(percent))


def SetLCD(
    backgroundColor: LCDColors = None,
    foregroundColor: LCDColors = None,
    bars: LCDBars = None,
    horizontalLine=-1,
    verticalLine=-1,
    contrast=-1,
):
    """Neato API Command: SetLCD.
    Sets the LCD to the specified display. (TestMode Only)

    Args:
        backgroundColor (LCDColors, optional): Fill LCD background with White or Black color. Defaults to None.
        foregroundColor (LCDColors, optional): Use White or Black as Foreground (line) color. Defaults to None.
        bars (LCDBars, optional): Draw alternating horizontal or vertical lines (FG,BG,FG,BG,...), across the whole screen. Defaults to None.
        horizontalLine (int, optional): Draw a horizontal line (in foreground color) at the specified row. Defaults to -1 (not sent).
        verticalLine (int, optional): Draw a vertical line (in foreground color) at the specified column. Defaults to -1 (not sent).
        contrast (int, optional): Set the following value as the LCD Contrast value into NAND. 0-63. Defaults to -1 (not sent).
    """

    __write(
        "SetLCD"
        + (" BG" + backgroundColor.value if backgroundColor else "")
        + (" FG" + foregroundColor.value if foregroundColor else "")
        + (" " + bars.value + "Bars" if bars else "")
        + (" HLine " + str(horizontalLine) if horizontalLine > -1 else "")
        + (" VLine " + str(verticalLine) if verticalLine > -1 else "")
        + (" Contrast " + str(contrast) if contrast > -1 else "")
    )


def SetLDSRotation(rotate):
    """Neato API Command: SetLDSRotation.
    Sets LDS rotation on or off. Can only be run in TestMode.

    Args:
        rotate (bool): Whether or not to have the Laser Rotate. True: Rotate, False:Stop rotating.
    """

    __write("SetLDSRotation " + ("On" if rotate else "Off"))


def SetLED(backlight: BacklightStatus = None, button: ButtonColors = None):
    """Neato API Command: SetLED.
    Sets the specified LED to on,off,blink, or dim. (TestMode Only)

    Args:
        backlight (BacklightStatus, optional): LCD Backlight Color. Defaults to None.
        button (ButtonColors, optional): Neato Button and LED Color. Defaults to None.
    """

    __write(
        "SetLED"
        + (" " + backlight.value if backlight else "")
        + (" " + button.value if button else "")
    )


def SetMotorBrush(enabled, rpm):
    """Neato API Command: SetMotor. Seperated from SetMotor for ease of use.
    Enabled or Disables brush motor and sets to run a requested speed. (TestMode Only)

    Args:
        enabled ([type]): Enable or disable Brush motor
        rpm (int, optional): RPM of the motor

    """

    __write(
        "SetMotor brush"
        + (" BrushEnable" if enabled else " BrushDisable")
        + (" RPM " + str(rpm) if rpm else "")
    )


def SetMotorVacuum(percent=None, rpm=None):
    """Neato API Command: SetMotor. Seperated from SetMotor for ease of use.
    Enabled or disables the vacuum motor. Calling the command wihtout supplying a value will turn off the Vacuum. Set the RPM or percent to turn on the vacumm to specified amount. Percent will be used if both are supplied.

    Args:
        rpm (int, optional): RPM of the vacuum motor. Defaults to None.
        percent (int, optional): Percent duty of the vacuum motor. Defaults to None.
    """

    vacuumOnOff = "VacuumOff"

    if rpm or percent:
        vacuumOnOff = "VacuumOn"

    __write("SetMotor " + vacuumOnOff)
    return


def SetMotorWheelsEnable(rWheel=True, lWheel=True):
    """Neato API Command: SetMotor. Seperated from SetMotor for ease of use.
    Enabled or disabled wheel motors. (TestMode Only)

    Args:
        rWheel (bool, optional): Enable or disable right wheel. Defaults to True.
        lWheel (bool, optional): Enable or disable left wheel. Defaults to True.
    """

    __write(
        "SetMotor"
        + (" RWheelEnable" if rWheel else " RWheelDisable")
        + (" LWheelEnable" if lWheel else " LWheelDisable")
    )


def SetMotorWheels(lWheelDist=0, rWheelDist=0, speed=1, accel=None):
    """Neato API Command: SetMotor. Seperated from SetMotor for ease of use.
    Sets the specified wheel motors to run in a direction at a requested speed. (TestMode Only)
    Args:
        lWheelDist (int, optional): [description]. Defaults to 0.
        rWheelDist (int, optional): [description]. Defaults to 0.
        speed (int, optional): [description]. Defaults to 1.
        accel ([type], optional): [description]. Defaults to None.
    """

    __write(
        "SetMotor"
        + " lWheelDist "
        + str(lWheelDist)
        + " rWheelDist "
        + str(rWheelDist)
        + " speed "
        + str(speed)
        + (" accel " + str(accel) if accel else "")
    )


def SetSchedule(
    day=0,
    hour=0,
    minute=0,
    scheduleType: ScheduleTypes = ScheduleTypes.House,
    enabled=True,
):
    """Neato API Command: SetSchedule.
    Modify Cleaning Schedule

    Args:
        day (int, optional): Day of week value Sunday=0,Monday=1. Defaults to 0.
        hour (int, optional): Hour value 0-23. Defaults to 0.
        minute (int, optional): Minutes value 0-59. Defaults to 0.
        scheduleType (ScheduleTypes, optional): The Schedule type to set. Defaults to ScheduleTypes.House.
        enabled (OnToggle, optional): Enable or Disable Scheduled cleanings. Defaults to OnToggle.On.
    """

    __write(
        "SetSchedule"
        + (" " + str(day))
        + (" " + str(hour))
        + (" " + str(minute))
        + (" " + scheduleType.value)
        + (" ON" if enabled else " OFF")
    )


def SetSystemMode(mode: SystemModes):
    """Neato API Command: SetSystemMode.
    Set the operation mode of the robot. (TestMode Only)

    Args:
        mode (SystemModes): The system mode to set.

    """

    __write("SetSystemMode " + mode.value)


def SetTime(now=datetime.datetime.today()):
    """Neato API Command: SetTime.
    Sets the current day, hour, and minute for the scheduler clock.

    Args:
        now (datetime.datetime, optional): supply a datetime to set to a specific datetime. Only day of week, hour, minute, and second are used. Defaults to now.

    """

    day = now.weekday() + 1
    if day == 8:
        day = 0

    __write(
        "SetTime"
        + " Day "
        + str(day)
        + " Hour "
        + str(now.hour)
        + " Min "
        + str(now.minute)
        + " Sec "
        + str(now.second)
    )


def SetWallFollower(enable):
    """Neato API Command: SetWallFollower.
        Enables/Disables wall follower

    Args:
        enable (bool): Enable or disable wall follower

    Returns:
        [type]: [description]
    """

    __write("SetWallFollower " + ("Enable" if enable else "Disable"))


def TestMode(on):
    """Neato API Command: TestMode.
    Sets TestMode on or off. Some commands can only be run in TestMode.

    Args:
        on (bool): Turns Testmode on or off.
    """
    __write("TestMode " + ("On" if on else "Off"))


def test():
    """Tests generates for testing module. Review code. Most are commented out but available for reference."""
    # TestMode(False)
    # TestMode(False)
    # Clean(CleanMode.House)
    # time.sleep(15)
    # Clean(CleanMode.Spot)
    # time.sleep(15)
    # Clean(CleanMode.Stop)
    # print(GetAccel())
    # print(GetAnalogSensors())
    # print(GetButtons())
    # print(GetCalInfo())
    # print(GetCharger())
    # print(GetDigitalSensors())
    # print(GetErr())
    # SetLDSRotation(True)
    # time.sleep(5)
    # print(GetLDSScan())
    # SetLDSRotation(False)
    # print(GetLifeStatLog())
    # print(GetMotors())
    # print(GetSchedule())
    # print(GetSysLog())
    print(GetTime())
    # print(SetTime())
    # print(GetTime())

    # print(GetVersion())
    # print(GetWarranty())
    # print(Help())

    # SetLED(BacklightStatus.On)
    # time.sleep(1)
    # SetLED(BacklightStatus.Off)
    # time.sleep(1)
    # SetLED(button=ButtonColors.Green)
    # time.sleep(1)
    # SetLED(button=ButtonColors.GreenDim)
    # time.sleep(1)
    # SetLED(button=ButtonColors.Amber)
    # time.sleep(1)
    # SetLED(button=ButtonColors.AmberDim)
    # time.sleep(1)
    # SetLED(button=ButtonColors.LEDGreen)
    # time.sleep(1)
    # SetLED(button=ButtonColors.LEDRed)
    # time.sleep(1)
    # SetLED(button=ButtonColors.Off)
    # time.sleep(1)
    # SetLED(BacklightStatus.On, ButtonColors.LEDGreen)
    # time.sleep(2)
    # SetLED(BacklightStatus.Off, ButtonColors.Off)

    # SetMotorVacuum(100)

    # PlaySound(Sounds.Alert)
    # SetSystemMode(SystemModes.Shutdown)
    # time.sleep(5)
    # PlaySound(Sounds.Alert)
    # SetSystemMode(SystemModes.Standby)
    # time.sleep(5)
    # PlaySound(Sounds.Alert)
    # SetSystemMode(SystemModes.PowerCycle)

    # i = 0
    # PlaySound()
    # while i <= 20:
    #     PlaySound(i)
    #     time.sleep(2)
    #     i += 1

    # SetMotorWheels(100, 100, 50)
    # SetMotorWheels(100, 100, 100, 5)

    # SetMotorVacuum(60)

    # SetMotorVacuum(None, 90)

    # SetMotorVacuum(None, 0)

    # SetMotorVacuum()

    # SetLCD(LCDColors.Black)
    # time.sleep(.5)
    # SetLCD(LCDColors.White)
    # time.sleep(.5)
    # i = 0
    # while i <= 125:
    #     SetLCD(horizontalLine=i)
    #     i += 2
    # i = 0
    # while i <= 125:
    #     SetLCD(verticalLine=i)
    #     i += 2
    # SetLCD(LCDColors.White)
    # i = 0
    # while i <= 125:
    #     SetLCD(verticalLine=i, horizontalLine=i)
    #     i += 2
    # # change background/foreground color
    # SetLCD(backgroundColor=LCDColors.Black, foregroundColor=LCDColors.White)
    # PlaySound(Sounds.Alert)
    # time.sleep(1)
    # i = 0
    # while i <= 120:
    #     SetLCD(horizontalLine=i)
    #     i += 5
    # SetLCD(backgroundColor=LCDColors.White, foregroundColor=LCDColors.Black)
    # PlaySound(Sounds.Alert)
    # time.sleep(1)
    # i = 0
    # while i <= 120:
    #     SetLCD(horizontalLine=i)
    #     i += 5
    # SetLCD(backgroundColor=LCDColors.White)
    # SetLCD(bars=LCDBars.Vertical)
    # PlaySound(Sounds.Alert)
    # time.sleep(1)
    # SetLCD(bars=LCDBars.Horizontal)
    # PlaySound(Sounds.Alert)
    # time.sleep(1)

    # print(GetSchedule())
    # SetSchedule(1, 1, 0, ScheduleTypes.House, True)
    # print(GetSchedule())
    # SetSchedule(1, 1, 0, ScheduleTypes.House, False)
    # print(GetSchedule())
    # SetSchedule(1, 1, 0, ScheduleTypes.None_, True)
    # print(GetSchedule())
    # SetSchedule(1, 1, 0, ScheduleTypes.None_, False)
    # print(GetSchedule())

    # DiagTest(TestModes.DrivePath, drivePathDist=100)
    # DiagTest()

    # Performance Tests

    # start = time.time()
    # i = 0
    # while i < 10:
    #     GetLDSScan()
    #     i += 1
    # end = time.time()
    # print("Total Time: " + str(end - start))
    # # Neato XV Signature Pro (XV28)
    # # using readline: 44.06494688987732
    # # using readlines: 0.5128560066223145

    # start = time.time()
    # i = 0
    # while i < 10:
    #     GetLDSScan()
    #     GetButtons()
    #     GetCharger()
    #     GetDigitalSensors()
    #     GetAnalogSensors()
    #     i += 1
    # end = time.time()
    # print("Total Time: " + str(end - start))
    # return
    # # Neato XV Signature Pro (XV28)
    # # 1x:   0.16377997398376465
    # # 10x:  1.5376250743865967
    # # 100x: 16.056003093719482
    return


def init(port, printDebug=False, timeout=0.5):
    """initilize connection with Neato.

    Args:
        port (string): port description such "/dev/neato" or "COM3"
        printDebug (bool, optional): Whether or not to print debug logging. Defaults to False.
    """
    global __serialPort, __debug, __timeout
    __debug = printDebug
    __serialPort = serial.Serial(port, timeout=0)
    __timeout = timeout
