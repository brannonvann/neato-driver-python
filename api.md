# API

This describes the `neato_driver.py` API. The documentation is ordered and grouped according to the Neato API which matches the `neato_driver.py` API in all but a few cases. Any differences are noted.

Each of the `neato_driver` functions are described below along with the Neato vacuum supplied help description that describes the function and if the function returns data, the data type returned and an example is provided.

This was generated using the `api.py` script. To produce the documentation, adjust the serial port to match your Neato's and run the script.


## Help

The below description from the Neato vacuum maps to the `neato_driver.Help()` function

```
Help - Without any argument, this prints a list of all possible cmds.
With a command name, it prints the help for that particular command
Cmd - (Optional) Next argument is command to show help for.
If Cmd option not used, help gives list of all commands.
```

Returns: `<class 'str'>`

**Data Example:**

```
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
```

## Clean

The below description from the Neato vacuum maps to the `neato_driver.Clean()` function

```
Clean - Starts a cleaning by simulating press of start button.
House - (Optional) Equivalent to pressing 'Start' button once.
Starts a house cleaning.
(House cleaning mode is the default cleaning mode.)
Spot - (Optional) Starts a spot clean.
Width - (Optional) Spot Width in CM (100-500)(-1=use default).
Height - (Optional) Spot Height in CM (100-500)(-1=use default).
Stop - Stop Cleaning.
```

## DiagTest

The below description from the Neato vacuum maps to the `neato_driver.DiagTest()` function

```
DiagTest - Executes different test modes. Once set, press Start button to engage. (Test modes are mutually exclusive.)
TestsOff - Stop Diagnostic Test and clears all diagnostic test modes.
DrivePath - Sets DrivePath TestMode. Press start button to start. Robot travels straight by commanded distance as path. Mutually exclusive with other diagtest modes. Use 'TestsOff' option to stop.
DriveForever - Sets DriveForever TestMode. Press start button to start. Robot drives continuously. Mutually exclusive with other diagtest modes. Use 'TestsOff' option to stop.
MoveAndBump - Sets Move and Bump TestMode. Press start button to start. Executes canned series of motions, but will react to bumps. Mutually exclusive with other diagtest modes.
DropTest - Enables DropTest. Robot drives forward until a drop is detected. Mutually exclusive with other diagtest modes.
AutoCycle - DropTest argument to enable automatic restart of the test. The robot will drive backwards and then forward until a drop is detected until the test is over.
OneShot - Only executes test once.
BrushOn - Turns on brush during test. May conflict with motor commands of test so use carefully!
VacuumOn - Turns on vacuum during test. May conflict with motor commands of test so use carefully!
LDSOn - Turns on LDS during test. May conflict with motor commands of test so use carefully!
SideBrushOn - Turns on side brush during test. May conflict with motor commands of test so use carefully!
AllMotorsOn - Turns on brush, vacuum, and lds during test. May conflict with motor commands of test so use carefully!
DisablePickupDetect - Ignores pickup (wheel suspension). By default, pickup detect is enabled and stops the test.
DrivePathDist - Distance in mm
DriveForeverLeftDist - Use next arg to set left wheel dist for DriveForever test. Requires DriveForeverRightDist as well. The ratio of this value to DriveForeverRightDist determines turn radius.
DriveForeverRightDist - Use next arg to set right wheel dist for DriveForever test. Requires DriveForeverLeftDist as well. The ratio of this value to DriveForeverLeftDist determines turn radius.
DriveForeverSpeed - Use next arg to set turn speed of outer wheel for DriveForever test in mm/s.
Speed - DropTest argument to set the robot speed in mm/s.
BrushSpeed - DropTest argument to set the speed of the brush in rpm.
```

## GetAccel

The below description from the Neato vacuum maps to the `neato_driver.GetAccel()` function

```
GetAccel - Get the Accelerometer readings.
brief - Returns a single-line summary. Data order:
Revision(0)
Pitch
Roll
X Axis
Y Axis
Z Axis
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'PitchInDegrees': -0.85, 'RollInDegrees': 0.91, 'XInG': -0.015, 'YInG': 0.016, 'ZInG': 1.007, 'SumInG': 1.007}
```

## GetAnalogSensors

The below description from the Neato vacuum maps to the `neato_driver.GetAnalogSensors()` function

```
GetAnalogSensors - Get the A2D readings for the analog sensors.
raw - Return raw analog sensor values as milliVolts.
(Default is sensor values in native units of what they measure.)
mainboard - Return raw analog sensor values as milliVolts from the mainboard.
(Default is sensor values in native units of what they measure.)
stats - Return stats (avg,max,min,dev,cnt) of raw analog sensor values as milliVolts.
(Implies 'raw' option)
brief - Returns a single-line summary of default values. Data order:
Revision(0)
Wall Sensor MM
Battery Voltage mV
Left Drop MM
Right Drop MM
Left Mag
Right Mag
Vacuum Current mA
Charge Voltage mV
Battery Temp 0 C
Battery Temp 1 C
Current mA
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'WallSensorInMM': 81, 'BatteryVoltageInmV': 4154, 'LeftDropInMM': 150, 'RightDropInMM': 150, 'LeftMagSensor': -236, 'RightMagSensor': -232, 'UIButtonInmV': 3335, 'VacuumCurrentInmA': 0, 'ChargeVoltInmV': 23938, 'BatteryTemp0InC': 25, 'BatteryTemp1InC': 23, 'CurrentInmA': 113, 'SideBrushCurrentInmA': 0, 'VoltageReferenceInmV': 1225, 'AccelXInmG': -16, 'AccelYInmG': 20, 'AccelZInmG': 1004}
```

## GetButtons

The below description from the Neato vacuum maps to the `neato_driver.GetButtons()` function

```
GetButtons - Get the state of the UI Buttons.
brief - Returns a single-line summary. Data order:
Revision(0)
Soft key
Up
Start
Back
Down
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'BTN_SOFT_KEY': False, 'BTN_SCROLL_UP': False, 'BTN_START': False, 'BTN_BACK': False, 'BTN_SCROLL_DOWN': False}
```

## GetCalInfo

The below description from the Neato vacuum maps to the `neato_driver.GetCalInfo()` function

```
GetCalInfo - Prints out the cal info from the System Control Block.
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'LDSOffset': 0, 'XAccel': 0, 'YAccel': 0, 'ZAccel': 0, 'RTCOffset': 0, 'LCDContrast': 25, 'RDropMin': 293, 'RDropMid': 168, 'RDropMax': 84, 'LDropMin': 295, 'LDropMid': 168, 'LDropMax': 68, 'WallMin': 721, 'WallMid': 265, 'WallMax': 140, 'QAState': 0, 'CleaningTestSurface': 'carpet', 'CleaningTestHardSpeed': 200, 'CleaningTestCarpetSpeed': 100, 'CleaningTestHardDistance': 1200, 'CleaningTestCarpetDistance': 1200}
```

## GetCharger

The below description from the Neato vacuum maps to the `neato_driver.GetCharger()` function

```
GetCharger - Get the diagnostic data for the charging system.
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'FuelPercent': 0, 'BatteryOverTemp': False, 'ChargingActive': False, 'ChargingEnabled': False, 'ConfidentOnFuel': False, 'OnReservedFuel': True, 'EmptyFuel': True, 'BatteryFailure': False, 'ExtPwrPresent': True, 'ThermistorPresent[0]': True, 'ThermistorPresent[1]': True, 'BattTempCAvg[0]': 25, 'BattTempCAvg[1]': 23, 'VBattV': 4.15, 'VExtV': 23.98, 'Charger_mAH': 0.0}
```

## GetDigitalSensors

The below description from the Neato vacuum maps to the `neato_driver.GetDigitalSensors()` function

```
GetDigitalSensors - Get the state of the digital sensors.
brief - Returns a single-line summary. Data order:
Revision(0)
DC Jack
Dustbin
Left Wheel
Right Wheel
Left Side Bumper
Left Front Bumper
Right Side Bumper
Right Front Bumper
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'SNSR_DC_JACK_CONNECT': False, 'SNSR_DUSTBIN_IS_IN': True, 'SNSR_LEFT_WHEEL_EXTENDED': False, 'SNSR_RIGHT_WHEEL_EXTENDED': False, 'LSIDEBIT': False, 'LFRONTBIT': False, 'RSIDEBIT': False, 'RFRONTBIT': False}
```

## GetErr

The below description from the Neato vacuum maps to the `neato_driver.GetErr()` function

```
GetErr - Get Error Message.
Clear - Dismiss the reported error.
```

Returns: `<class 'str'>`

**Data Example:**

```
GetErr
```

## GetLDSScan

The below description from the Neato vacuum maps to the `neato_driver.GetLDSScan()` function

```
GetLDSScan - Get scan packet from LDS.
streamOn - Start streaming of raw (binary) LDS readings
streamOff - Stop streaming of raw LDS readings
```

Returns: `<class 'dict'>`

**Data Example:**

```
{0: [622, 297, 0], 1: [0, 0, 8035], 2: [0, 0, 8021], 3: [614, 460, 0], 4: [619, 283, 0], 5: [17010, 23, 0], 6: [0, 0, 8035], 7: [0, 0, 8035], 8: [0, 0, 8035], 9: [0, 0, 8035], 10: [0, 0, 8035], 11: [0, 0, 8003], 12: [0, 0, 8035], 13: [4915, 5, 0], 14: [5933, 7, 0], 15: [0, 0, 8035], 16: [0, 0, 8035], 17: [0, 0, 8035], 18: [0, 0, 8035], 19: [0, 0, 8035], 20: [0, 0, 8003], 21: [0, 0, 8035], 22: [0, 0, 8035], 23: [5126, 5, 0], 24: [0, 0, 8035], 25: [0, 0, 8035], 26: [0, 0, 8035], 27: [0, 0, 8025], 28: [0, 0, 8035], 29: [0, 0, 8035], 30: [0, 0, 8035], 31: [0, 0, 8035], 32: [0, 0, 8025], 33: [0, 0, 8035], 34: [0, 0, 8035], 35: [0, 0, 8035], 36: [0, 0, 8035], 37: [0, 0, 8035], 38: [0, 0, 8025], 39: [0, 0, 8035], 40: [0, 0, 8035], 41: [0, 0, 8035], 42: [0, 0, 8035], 43: [0, 0, 8025], 44: [3407, 11, 0], 45: [3431, 9, 0], 46: [3492, 9, 0], 47: [3599, 8, 0], 48: [3611, 8, 0], 49: [3671, 7, 0], 50: [0, 7, 8021], 51: [0, 0, 8035], 52: [0, 0, 8035], 53: [0, 0, 8035], 54: [0, 0, 8035], 55: [0, 0, 8025], 56: [0, 0, 8035], 57: [4693, 6, 0], 58: [0, 0, 8035], 59: [0, 0, 8035], 60: [0, 0, 8035], 61: [0, 0, 8035], 62: [0, 0, 8025], 63: [0, 0, 8035], 64: [5027, 6, 0], 65: [0, 0, 8035], 66: [0, 0, 8035], 67: [4863, 5, 0], 68: [0, 0, 8002], 69: [4937, 10, 0], 70: [4876, 7, 0], 71: [4846, 9, 0], 72: [4791, 7, 0], 73: [4762, 13, 0], 74: [0, 0, 8035], 75: [2683, 17, 0], 76: [2657, 24, 0], 77: [2628, 22, 0], 78: [2550, 21, 0], 79: [0, 0, 8035], 80: [0, 0, 8035], 81: [0, 0, 8035], 82: [0, 0, 8035], 83: [0, 0, 8025], 84: [0, 0, 8035], 85: [0, 0, 8035], 86: [0, 0, 8035], 87: [0, 0, 8035], 88: [0, 0, 8003], 89: [0, 0, 8035], 90: [0, 0, 8035], 91: [1071, 6, 0], 92: [0, 0, 8035], 93: [0, 0, 8035], 94: [0, 0, 8035], 95: [0, 0, 8035], 96: [0, 0, 8025], 97: [0, 0, 8035], 98: [0, 0, 8035], 99: [0, 0, 8035], 100: [0, 0, 8035], 101: [0, 0, 8035], 102: [0, 0, 8035], 103: [0, 0, 8035], 104: [0, 0, 8025], 105: [0, 0, 8035], 106: [0, 0, 8035], 107: [0, 0, 8035], 108: [0, 0, 8035], 109: [0, 0, 8035], 110: [0, 0, 8025], 111: [0, 0, 8035], 112: [0, 0, 8035], 113: [0, 0, 8035], 114: [0, 0, 8035], 115: [0, 0, 8002], 116: [0, 0, 8035], 117: [0, 0, 8035], 118: [0, 0, 8035], 119: [0, 0, 8035], 120: [0, 0, 8035], 121: [0, 0, 8025], 122: [0, 0, 8035], 123: [0, 0, 8035], 124: [0, 0, 8035], 125: [0, 0, 8035], 126: [0, 0, 8035], 127: [0, 0, 8003], 128: [0, 0, 8035], 129: [0, 0, 8035], 130: [0, 0, 8035], 131: [0, 0, 8035], 132: [0, 0, 8025], 133: [0, 0, 8035], 134: [0, 0, 8035], 135: [0, 0, 8035], 136: [0, 0, 8035], 137: [0, 0, 8035], 138: [0, 0, 8025], 139: [0, 0, 8035], 140: [0, 0, 8035], 141: [0, 0, 8035], 142: [0, 0, 8035], 143: [0, 0, 8025], 144: [0, 0, 8035], 145: [0, 0, 8035], 146: [0, 0, 8021], 147: [0, 0, 8035], 148: [0, 0, 8035], 149: [0, 0, 8035], 150: [0, 0, 8002], 151: [0, 0, 8035], 152: [0, 0, 8035], 153: [0, 0, 8035], 154: [0, 0, 8035], 155: [0, 0, 8003], 156: [0, 0, 8035], 157: [0, 0, 8035], 158: [0, 0, 8035], 159: [0, 0, 8025], 160: [0, 0, 8035], 161: [0, 0, 8035], 162: [0, 0, 8035], 163: [0, 0, 8035], 164: [0, 0, 8025], 165: [0, 0, 8035], 166: [0, 0, 8035], 167: [0, 0, 8035], 168: [0, 0, 8035], 169: [0, 0, 8025], 170: [0, 0, 8035], 171: [0, 0, 8035], 172: [0, 0, 8035], 173: [0, 0, 8035], 174: [0, 0, 8025], 175: [0, 0, 8035], 176: [0, 0, 8035], 177: [0, 0, 8035], 178: [0, 0, 8035], 179: [0, 0, 8025], 180: [0, 0, 8035], 181: [0, 0, 8035], 182: [0, 0, 8035], 183: [0, 0, 8035], 184: [0, 0, 8003], 185: [0, 0, 8035], 186: [0, 0, 8035], 187: [0, 0, 8035], 188: [0, 0, 8035], 189: [0, 0, 8025], 190: [0, 0, 8035], 191: [0, 0, 8035], 192: [0, 0, 8035], 193: [0, 0, 8035], 194: [0, 0, 8035], 195: [0, 0, 8035], 196: [0, 0, 8025], 197: [0, 0, 8035], 198: [0, 0, 8035], 199: [0, 0, 8035], 200: [0, 0, 8035], 201: [0, 0, 8035], 202: [0, 0, 8003], 203: [0, 0, 8035], 204: [0, 0, 8035], 205: [0, 0, 8035], 206: [0, 0, 8035], 207: [0, 0, 8025], 208: [0, 0, 8035], 209: [0, 0, 8035], 210: [0, 0, 8035], 211: [0, 0, 8035], 212: [0, 0, 8003], 213: [0, 0, 8035], 214: [0, 0, 8035], 215: [0, 0, 8035], 216: [0, 0, 8035], 217: [0, 0, 8025], 218: [0, 0, 8035], 219: [0, 0, 8035], 220: [0, 0, 8035], 221: [0, 0, 8035], 222: [0, 0, 8002], 223: [0, 0, 8035], 224: [0, 0, 8035], 225: [0, 0, 8035], 226: [0, 0, 8035], 227: [0, 0, 8025], 228: [0, 0, 8035], 229: [0, 0, 8035], 230: [0, 0, 8035], 231: [0, 0, 8035], 232: [0, 0, 8003], 233: [0, 0, 8035], 234: [0, 0, 8035], 235: [0, 0, 8035], 236: [0, 0, 8035], 237: [0, 0, 8025], 238: [0, 0, 8035], 239: [0, 0, 8035], 240: [0, 0, 8035], 241: [698, 179, 0], 242: [0, 179, 8021], 243: [676, 386, 0], 244: [669, 398, 0], 245: [663, 422, 0], 246: [657, 396, 0], 247: [650, 379, 0], 248: [638, 261, 0], 249: [625, 45, 0], 250: [0, 0, 8035], 251: [0, 0, 8035], 252: [0, 0, 8035], 253: [0, 0, 8035], 254: [0, 0, 8025], 255: [0, 0, 8035], 256: [0, 0, 8035], 257: [17008, 29, 0], 258: [616, 226, 0], 259: [607, 423, 0], 260: [602, 532, 0], 261: [599, 517, 0], 262: [597, 585, 0], 263: [595, 583, 0], 264: [594, 562, 0], 265: [592, 589, 0], 266: [590, 603, 0], 267: [590, 596, 0], 268: [589, 644, 0], 269: [588, 627, 0], 270: [587, 675, 0], 271: [587, 666, 0], 272: [587, 689, 0], 273: [587, 732, 0], 274: [587, 700, 0], 275: [588, 714, 0], 276: [588, 691, 0], 277: [589, 641, 0], 278: [590, 626, 0], 279: [591, 616, 0], 280: [593, 557, 0], 281: [598, 416, 0], 282: [604, 343, 0], 283: [609, 246, 0], 284: [614, 89, 0], 285: [0, 0, 8035], 286: [0, 0, 8035], 287: [0, 0, 8035], 288: [0, 0, 8035], 289: [0, 0, 8025], 290: [490, 275, 0], 291: [502, 316, 0], 292: [600, 453, 0], 293: [578, 66, 0], 294: [543, 86, 0], 295: [524, 178, 0], 296: [521, 528, 0], 297: [517, 683, 0], 298: [515, 744, 0], 299: [515, 695, 0], 300: [519, 650, 0], 301: [567, 500, 0], 302: [559, 550, 0], 303: [0, 0, 8035], 304: [0, 0, 8035], 305: [16771, 25, 0], 306: [0, 0, 8035], 307: [0, 0, 8035], 308: [0, 0, 8035], 309: [0, 0, 8035], 310: [0, 0, 8035], 311: [0, 0, 8025], 312: [0, 0, 8035], 313: [0, 0, 8035], 314: [0, 0, 8021], 315: [0, 0, 8035], 316: [2477, 10, 0], 317: [0, 0, 8002], 318: [0, 0, 8035], 319: [0, 0, 8035], 320: [523, 191, 0], 321: [0, 0, 8035], 322: [498, 677, 0], 323: [492, 768, 0], 324: [485, 612, 0], 325: [475, 147, 0], 326: [0, 0, 8035], 327: [0, 0, 8035], 328: [0, 0, 8035], 329: [0, 0, 8035], 330: [0, 0, 8025], 331: [16908, 19, 0], 332: [16920, 22, 0], 333: [532, 396, 0], 334: [542, 150, 0], 335: [1196, 53, 0], 336: [1153, 56, 0], 337: [1141, 28, 0], 338: [0, 0, 8035], 339: [1186, 21, 0], 340: [1226, 55, 0], 341: [1326, 98, 0], 342: [1081, 33, 0], 343: [0, 0, 8035], 344: [0, 0, 8035], 345: [0, 0, 8035], 346: [0, 0, 8035], 347: [0, 0, 8025], 348: [0, 0, 8035], 349: [0, 0, 8035], 350: [0, 0, 8035], 351: [0, 0, 8035], 352: [0, 0, 8025], 353: [1724, 57, 0], 354: [1562, 26, 0], 355: [1579, 15, 0], 356: [1571, 64, 0], 357: [0, 0, 8035], 358: [0, 0, 8035], 359: [0, 0, 8035], 'ROTATION_SPEED': 5.12}
```

## GetMotors

The below description from the Neato vacuum maps to the `neato_driver.GetMotors()` function

```
GetMotors - Get the diagnostic data for the motors.
Brush - Return Brush Motor stats.
Vacuum - Return Vacuum Motor stats.
LeftWheel - Return LeftWheel Motor stats.
RightWheel - Return RightWheel Motor stats.
Laser - Return LDS Motor stats.
Charger - Return Battery Charger stats.
SideBrush - Return Side Brush stats.
brief - Returns a single-line WHEEL ONLY summary. Data order:
Revision(0)
Left Wheel RPM
Left Wheel Position MM
Left Wheel Speed
Right Wheel RPM
Right Wheel Position MM
Right Wheel Speed
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'Brush_RPM': 0, 'Brush_mA': 0, 'Vacuum_RPM': 0, 'Vacuum_mA': 0, 'LeftWheel_RPM': 0, 'LeftWheel_Load%': 0, 'LeftWheel_PositionInMM': 2219, 'LeftWheel_Speed': 0, 'RightWheel_RPM': 0, 'RightWheel_Load%': 0, 'RightWheel_PositionInMM': 1818, 'RightWheel_Speed': 0, 'Charger_mAH': 0, 'SideBrush_mA': 0}
```

## GetSchedule

The below description from the Neato vacuum maps to the `neato_driver.GetSchedule()` function

```
GetSchedule - Get the Cleaning Schedule. (24 hour clock format)
Day - Day of the week to get schedule for. Sun=0,Sat=6.
If not specified, then all days are given.
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'schedule': 'disabled', 'Sun': {'start': '00:00', 'flag': 'None'}, 'Mon': {'start': '00:00', 'flag': 'None'}, 'Tue': {'start': '00:00', 'flag': 'None'}, 'Wed': {'start': '08:15', 'flag': 'H'}, 'Thu': {'start': '00:00', 'flag': 'None'}, 'Fri': {'start': '08:15', 'flag': 'H'}, 'Sat': {'start': '00:00', 'flag': 'None'}}
```

## GetTime

The below description from the Neato vacuum maps to the `neato_driver.GetTime()` function

```
GetTime - Get Current Scheduler Time.
```

Returns: `<class 'datetime.datetime'>`

**Data Example:**

```
2021-04-30 13:43:42.379894
```

## GetVersion

The below description from the Neato vacuum maps to the `neato_driver.GetVersion()` function

```
GetVersion - Get the version information for the system software and hardware.
```

Returns: `<class 'dict'>`

**Data Example:**

```
{'Component': ['Major', 'Minor'], 'ModelID': ['-1', 'XV28'], 'ConfigID': ['1', ''], 'Serial Number': 'SERIAL-EXCLUDED', 'Software': ['3', '4'], 'BatteryType': ['1', 'NIMH_12CELL'], 'BlowerType': ['1', 'BLOWER_ORIG'], 'BrushSpeed': ['1200', ''], 'BrushMotorType': ['1', 'BRUSH_MOTOR_ORIG'], 'SideBrushType': ['1', 'SIDE_BRUSH_NONE'], 'WheelPodType': ['1', 'WHEEL_POD_ORIG'], 'DropSensorType': ['1', 'DROP_SENSOR_ORIG'], 'MagSensorType': ['1', 'MAG_SENSOR_ORIG'], 'WallSensorType': ['1', 'WALL_SENSOR_ORIG'], 'Locale': ['1', 'LOCALE_USA'], 'LDS Software': ['V2.6.15295', '0000000000'], 'LDS Serial': 'SERIAL-EXCLUDED', 'LDS CPU': ['F2802x/c001', ''], 'MainBoard Vendor ID': ['505', ''], 'MainBoard Serial Number': 'SERIAL-EXCLUDED', 'BootLoader Software': ['18119', 'P'], 'MainBoard Software': ['23179', '1'], 'MainBoard Boot': ['16219', ''], 'MainBoard Version': ['4', '0'], 'ChassisRev': ['2', '']}
```

## GetWarranty

The below description from the Neato vacuum maps to the `neato_driver.GetWarranty()` function

```
GetWarranty - Get the warranty validation codes.
```

Returns: `<class 'list'>`

**Data Example:**

```
['000ae7ee', '0137', '18244b76']
```

## PlaySound

The below description from the Neato vacuum maps to the `neato_driver.PlaySound()` function

```
PlaySound - Play the specified sound in the robot.
SoundID - Play the sound library entry specified by the number in the next argument.
Legal values are:
0 - Waking Up
1 - Starting Cleaning
2 - Cleaning Completed
3 - Attention Needed
4 - Backing up into base station
5 - Base Station Docking Completed
6 - Test Sound 1
7 - Test Sound 2
8 - Test Sound 3
9 - Test Sound 4
10 - Test Sound 5
11 - Exploring
12 - ShutDown
13 - Picked Up
14 - Going to sleep
15 - Returning Home
16 - User Canceled Cleaning
17 - User Terminated Cleaning
18 - Slipped Off Base While Charging
19 - Alert
20 - Thank You
Stop - Stop playing sound.
```

## RestoreDefaults

The below description from the Neato vacuum maps to the `neato_driver.RestoreDefaults()` function

```
RestoreDefaults - Restore user settings to default.
```

## SetFuelGauge

The below description from the Neato vacuum maps to the `neato_driver.SetFuelGauge()` function

```
SetFuelGauge - Set Fuel Gauge Level.
Percent - Fuel Gauge percent from 0 to 100
```

## SetMotorBrush, SetMotorVacuum, SetMotorWheelsEnable, and SetMotorWheels

The below description from the Neato vacuum maps to the `neato_driver.`SetMotorBrush()`, `neato_driver.SetMotorVacuum()`, `neato_driver.SetMotorWheelsEnable()`, and `neato_driver.SetMotorWheels()` functions. These were divided to make it easier to integrate to the Neato.

```
SetMotor - Sets the specified motor to run in a direction at a requested speed. (TestMode Only)
LWheelDist - Distance in millimeters to drive Left wheel
+/- 10000, pos = forward, neg = backward
RWheelDist - Distance in millimeters to drive Right wheel
+/- 10000, pos = forward, neg = backward
Speed - Speed in millimeters/second
0-300, required for wheel movements
Accel - Acceleration in millimeters/second,
0-300, used only for wheel movements, defaults to 'Speed'
RPM - The following argument is the RPM of the motor.
0-10000, not used for wheels, but applied to all other motors
Brush - Brush motor (Mutually exclusive with wheels and vacuum.)
VacuumOn - Vacuum motor on (Mutually exclusive with VacuumOff)
VacuumOff - Vacuum motor off (Mutually exclusive with VacuumOn)
VacuumSpeed - Vacuum speed in percent (1-100)
RWheelDisable - Disable Right Wheel motor
LWheelDisable - Disable Left Wheel motor
BrushDisable - Disable Brush motor
RWheelEnable - Enable Right Wheel motor
LWheelEnable - Enable Left Wheel motor
BrushEnable - Enable Brush motor
SideBrushEnable - Enable Side Brush Motor motor
SideBrushDisable - Disable Side Brush Motor motor
SideBrushOn - Enable the Side Brush
SideBrushOff - Disable the Side Brush
SideBrushPower - Side Brush maximum power in milliwatts
```

## SetTime

The below description from the Neato vacuum maps to the `neato_driver.SetTime()` function

```
SetTime - Sets the current day, hour, and minute for the scheduler clock.
Day - Day of week value Sunday=0,Monday=1,... (required)
Hour - Hour value 0..23 (required)
Min - Minutes value 0..59 (required)
Sec - Seconds value 0..59 (Optional, defaults to 0)
```

## SetLED

The below description from the Neato vacuum maps to the `neato_driver.SetLED()` function

```
SetLED - Sets the specified LED to on,off,blink, or dim. (TestMode Only)
BacklightOn - LCD Backlight On  (mutually exclusive of BacklightOff)
BacklightOff - LCD Backlight Off (mutually exclusive of BacklightOn)
ButtonAmber - Start Button Amber (mutually exclusive of other Button options)
ButtonGreen - Start Button Green (mutually exclusive of other Button options)
LEDRed - Start Red LED (mutually exclusive of other Button options)
LEDGreen - Start Green LED (mutually exclusive of other Button options)
ButtonAmberDim - Start Button Amber Dim (mutually exclusive of other Button options)
ButtonGreenDim - Start Button Green Dim (mutually exclusive of other Button options)
ButtonOff - Start Button Off
BlinkOn - Start the LED Blink
BlinkOff - Stop the LED Blink
```

## SetIEC

The SetIEC function is not supported by this driver.

```
SetIEC - Sets the IEC Cleaning Test parameters
FloorSelection - Next Arg is the floor type < carpet | hard >
CarpetSpeed - Next Arg is test speed on carpet (10-300mm/s)
HardSpeed - Next Arg is test speed on hard floors (10-300mm/s)
Distance - Next Arg is test distance (200-4000 mm, default 1200)
```

## SetLCD

The below description from the Neato vacuum maps to the `neato_driver.SetLCD()` function

```
SetLCD - Sets the LCD to the specified display. (TestMode Only)
BGWhite - Fill LCD background with White
BGBlack - Fill LCD background with Black
HLine - Draw a horizontal line (in foreground color) at the following row.
VLine - Draw a vertical line (in foreground color) at the following column.
HBars - Draw alternating horizontal lines (FG,BG,FG,BG,...),
across the whole screen.
VBars - Draw alternating vertical lines (FG,BG,FG,BG,...),
across the whole screen.
FGWhite - Use White as Foreground (line) color
FGBlack - Use Black as Foreground (line) color
Contrast - Set the following value as the LCD Contrast value into NAND. 0..63
```

## SetLDSRotation

The below description from the Neato vacuum maps to the `neato_driver.SetLDSRotation()` function

```
SetLDSRotation - Sets LDS rotation on or off. Can only be run in TestMode.
On - Turns LDS rotation on. Mutually exclusive with Off.
Off - Turns LDS rotation off. Mutually exclusive with On.
```

## SetSchedule

The below description from the Neato vacuum maps to the `neato_driver.SetSchedule()` function

```
SetSchedule - Modify Cleaning Schedule.
Day - Day of the week to schedule cleaning for. Sun=0,Sat=6. (required)
Hour - Hour value 0..23 (required)
Min - Minutes value 0..59 (required)
House - Schedule to Clean whole house (default)
(Mutually exclusive with None)
None - Remove Scheduled Cleaning for specified day. Time is ignored.
(Mutually exclusive with House)
ON - Enable Scheduled cleanings (Mutually exclusive with OFF)
OFF - Disable Scheduled cleanings (Mutually exclusive with ON)
```

## SetSystemMode

The below description from the Neato vacuum maps to the `neato_driver.SetSystemMode()` function

```
SetSystemMode - Set the operation mode of the robot. (TestMode Only)
Shutdown - Shut down the robot. (mutually exclusive of other options)
Hibernate - Start hibernate operation.(mutually exclusive of other options)
Standby - Start standby operation. (mutually exclusive of other options)
PowerCycle - Power cycles the entire system. (mutually exclusive of other options)
```

## TestMode

The below description from the Neato vacuum maps to the `neato_driver.TestMode()` function

```
TestMode - Sets TestMode on or off. Some commands can only be run in TestMode.
On - Turns Testmode on. Mutually exclusive with Off.
Off - Turns Testmode off. Mutually exclusive with On.
```

## Upload

The Upload function is not supported by this driver.

```
Upload - Uploads new program to the robot.
code - Upload file is the main application. (Mutually exclusive with sound, LDS)
sound - Upload file is a sound module. (Mutually exclusive with code, LDS)
LDS - Upload file is an LDS module. (Mutually exclusive with sound, code)
mainboard - Upload file is mainboard module. (Mutually exclusive with sound, code and LDS)
size - data size to upload to device.
noburn - test option -- do NOT burn the flash after the upload.
readflash - test option -- read the flash at the current region.
reboot - Reset the robot after performing the upload.
```