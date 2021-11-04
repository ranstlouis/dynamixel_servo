# dynamixel_servo
A program to run a Dynamixel AX-12A servo motor
Built from [cbteeple's repo] (https://github.com/cbteeple/dynamixel_servo)

## Dependencies:
- [HalfDuplexSerial-for-Arduino](https://github.com/akira215/HalfDuplexSerial-for-Arduino) library (_required_) - For communicating with the servos via half-duplex serial
- [DynamixelArduino](https://github.com/akira215/DynamixelArduino) library (_required_) - For controlling dynamixel servos as nice objects.
- [Ax-12A-servo-library](https://github.com/jumejume1/AX-12A-servo-library) library (_optional_) - For connecting to servos via hardware serial (used for changing baud rates)

## Setup
1. Use "DynamixelBaudSwitch" to switch the baud rate of your servo to **57600**.
	- Be sure to connect the data line of the servo to the TX pin on your arduino.
	- Upload the program to the arduino. That's it.
2. Upload the "Dynamixel_Control_single" firmware.
	1. Define the comm pin to use (since dynamixels can be daisy-chained). This pin needs to have interrupt capabillities to work correctly.
	2. Define servo ID (This is the internal ID of the servo. Default is 1.)
	
	```cpp
	int servo_comm_pin = 2;
	int servo_id = 1;
	```
	3. Upload the program to an Arduino (I used an Arduino Nano).
	4. Set up your servo with 12V power and reset the arduino (power cycle).
## Usage
Send serial commands using a `[COMMAND];[ARG];[ARG]...` structure. This struture is described in detail in the [documentation for Ctrl-P](https://ctrl-p.cbteeple.com/latest/firmware/firmware_commands), but this servo program uses a different command set.

## Commands

### Main Commands:
- **SET** - Set the position setpoints in whatever units you are using (see below).
  - **SET;_[#1];[#2]_** - Set the position of all servos to the same value. #1 is a dummy number for compatibllity, #2 (`float`) is the servo position.
- **SPEED** - Set the speed of the servos (in direct units)
  - **SPEED;_[#1]_** - Set the speed of the servos to one value. #1 (`int`) is the desired speed.
  - **SPEED;_[#1]_** - Set the speed of the servos. #1 (`int`) is the desired speeds. 
- **TORQUE** - Set the torque of the servos (in direct units)
  - **TORQUE;_[#1]_** - Set the torque of the servos to one value. #1 (`int`) is the desired torque.
  
- **CONT** - Set the continuous operation mode (i.e. torque-determined stop vs. continuous torque)
  - **CONT;_[#1]_** - Set the continuous operation mode of the servos to one value. #1 (`bool`) is the desired mode.

### Safety Commands:
- **MAX** - Set the maximum position of the servos (in direct units)
  - **MAX;_[#1]_** - Set the maximum position of the servos to one value. #1 (`int`) is the desired maximum position.
 
- **MIN** - Set the maximum position of the servos (in direct units)
  - **MIN;_[#1]_** - Set the minimum position of the servos to one value. #1 (`int`) is the desired minimum position.
  

### Communication Commands
- **ON** - Turn on live data output
  - **ON** - (No Args)
- **OFF** - Turn off live data output
  - **OFF** - (No Args)
- **UNITS** - Set the output units 
  - **UNITS;_[#1]_** - #1 (`int`) is the unit type: (0 = direct servo units, 1 = angle (degrees), 2 = angle (radians), 3 = gripper width (mm)). Gripper width units are calculated for the [Trossen Phantom Parallel Gripper](https://www.trossenrobotics.com/p/phantomx-parallel-ax12-gripper.aspx)
- **ECHO** - Set the state of command echos
  - **ECHO;_[#1]_** - #1 is the echo state (0=off, 1=on)

### Servo Setup Commands
- **REBOOT** - Reboot a servo
  - **REBOOT;_[#1]_** - #1 (`int`) is the ID of the servo to reboot.
 
### Servo Diagnostics
- **PING** - Ping a servo
  - **PING;_[#1]_** - #1 (`int`) is the ID of the servo
- **VOLT** - Get the voltage (in V) of all servos
  - **VOLT** - (No Args)
- **TEMP** - Get the temperature (in C) of all servos
  - **TEMP** - (No Args)
- **FRIMWARE** - Get the firmware number of all servos
  - **FRIMWARE** - (No Args)
- **MODEL** - Get the model number of all servos
  - **MODEL** - (No Args)
  
### Other Commands:
- **LED** - Set the LED state of a servo
  - **LED;_[#1]_** - #1 (`int`) is the ID of the servo, #2 (`bool`) is the LED state (0=off, 1=on).


