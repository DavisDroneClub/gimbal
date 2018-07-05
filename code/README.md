# Gimbal Code V2.0 #

## File Descriptions ##
* code.ino: Program variable declaration, setup, and main loop.
* communication.ino: Functions for I2C and Serial communication.
* eeprom.ino: Functions for EEPROM data storage.
* functions.ino: General functions for operating gimbal.
* mpu6050.ino: Functions for reading and processing data from MPU6050

## Libraries ##
* PID_v1 library can be downloaded from https://github.com/br3ttb/Arduino-PID-Library
* Wire, Servo, and EEPROM library is preinstalled with Arduino IDE

## Upload Procedure ##
1. Download gimbal/code folder.
2. Open code.ino with Arduino IDE. (Available here: https://www.arduino.cc/en/Main/Software?)
3. Set board to Arduino Nano and select appropriate serial port.
4. Upload to board.

## Serial Communication ##
Serial communication can be used to read and write data to the gimbal. Command packets are sent from a computer to the gimbal, and data packets are sent from the gimbal to a computer.
Packets consist of a header (e.g. TUNEP) and data, separated by a semicolon. Data within a packet can also be split with a comma delimiter.

### Command Packets ###
Header | Data Format | Usage | Example
-------|-------------|-------|--------
TUNEP|float|Set Kp to the given float|TUNEP;0.1
TUNEI|float|Set Ki to the given float|TUNEI;0.1
TUNED|float|Set Kd to the given float|TUNED;0.1
SETSP|float|Set the setpoint of the PID controller (typically 0)|SETSP;0
SETDB|float|Set the deadband size in degrees|SETSP;2.5
SETDR|bool |Set gimbal direction. 0 for inverted, 1 for regular|SETDR;0
SETPT|bool |Enable/disable data packet. 0 for disable, 1 for enable|SETPT;1
GETEP|0    |Request an EEPR packet|GETEP;0

### Data Packets ###
Header | Data Format | Usage | Example
-------|-------------|-------|--------
WARN|Warning (text)|Issued when unknown command received | WARN;Unknown mode
LOOP|Loop time (int)|Issued when loop time exceeded, returns loop time in us | LOOP;4871
DATA|angle (float), loop duration (int), current time (int)|Returns angle in degrees, loop duration and current time in us | DATA;1.3;2320;13523
EEPR|Kp (float), Ki (float), Kd (float), deadband (float)|Returns data stored in EEPROM|EEPR;0.1,3,0.0015,2.0
