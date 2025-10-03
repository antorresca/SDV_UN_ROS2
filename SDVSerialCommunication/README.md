# SDV_Serial_Communication

SDV_Serial_Communication is a C++ firmware code, developed in Platformio IDE, intended to be used in multiple boards like Arduino, Tiva C or Launchpad EXP-MSP430F5529LP.

With this software, selected board converts in a "Serial Command Server" that controls motors of SDVUNx, read sensors and send data sensors throught UART port.

## How to use with an IDE

### Platformio IDE
It's recomended to use Platformio IDE and VScode. This IDE makes development of embed software easier. Copy or link main.cpp and *.h* or *.cpp* files to src folder. Also, copy or link the *platformio.ini* file: it contains necesary libraries and enviroment settings for every suported board.

An example of directory tree using Platformio IDE may look like this:

```
|--SDV_Serial_Communication_Tiva
   |--.pio
   |  |- ...
   |--.vscode
   |  |- ...
   |--include
   |  |- ...
   |--lib
   |  |- ...
   |--src
   |  |- main.cpp
   |  |- tiva_pins.h
   |  |- ...
   |--test
   |- .gitignore
   |- platformio.ini
```

### Arduino or Energia
Rename main.cpp with the name of container folder and replace extension .cpp with .ino. Copy or link all header and cpp files in container folder.

You may download third party libraries. Read *platformio.ini* file to check used libraries in this project.

An example of directory tree using Arduino IDE or Energia IDE may be like this:
```
|--SDV_Serial_Communication_Nano
   |- main.cpp
   |- ...
```


## Build for a specific board
Change pins header in include zone. Arduino Nano and Tiva C are full suported.

## Build and Upload the sketch using PlatformIO CLI
- Install PlatformIO CLI (visit its webpage)
- Go to folder project
- Check that *platformio.ini* file has correct settings for target board. Take note of *env* value: is configures the target board (Tiva or Atmega328).
- Check that PlatformIO can build the project, using *env* argument finded in *platformio.ini* file. Use the next command:
   ```
   platformio run --environment <env>
   ```
- If last command works, upload the sketch using the next command:
   ```
   platformio run --target upload --environment <env>
   ```

## Serial Operation
Connect to board using *PUTTY* or *cu* terminal program. If you want to use the board with an interactive terminal, read below *Command Summary*.

In ROS, the *sdv_un_ros* metapackage contains a node named *sdv_serial* that configurates the board, control the motors and read all SDV peripherals.

## Command feedback: codes and values
The next list shows every code that the Tiva board can print and the meaning of every field:

- 0 -> Motor CMD feedback: Used as confirmation
  - sample: "<0 m"
  - code: 0
  - values:
    - 0: command code
    - m: motor

- 1 -> IMU Data: Sends 13 values
  - sample: "<1 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 1.00"
  - code: 1
  - values:
    - 1: IMU data code
    - 1_float: Acceleration X axis
    - 2_float: Acceleration Y axis
    - 3_float: Acceleration Z axis
    - 4_float: Angular velocity in X axis
    - 5_float: Angular velocity in Y axis
    - 6_float: Angular velocity in Z axis
    - 7-float: Magnetic field in X axis
    - 8_float: Magnetic field in Y axis
    - 9_float: Magnetic field in Z axis
    - 10_float: Orientation in quaternion format, X axis
    - 11_float: Orientation in quaternion format, Y axis
    - 12_float: Orientation in quaternion format, Z axis
    - 13_float: Orientation in quaternion format, W axis

- 2 -> Odometry Data: Contains meassured speed for every motor. The length can be diferent, depending of number of motors.
  - sample: "<2 0.1 0.1"
  - code: 2
  - values:
    - 2: Odometry data code
    - 1_float: left motor for diferential robots or back_left motor for mecanum robots
    - 2_float: right motor for diferential robots or back_right motor for mecanum robots
    - 3_float: front_left motor for mecanum robots
    - 4_float: front_right motor for mecanum robots

- 3 -> SDV Status Data: Not yet implemented
  - code: 3

- 4 -> Ultrasound sensors Data: Contains meassured values from six Ultrasound sensors.
  - sample: "<4 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000"
  - code: 4
  - values:
    - 4: Ultrasound data code
    - 1_float: Ultrasound sensor 1
    - 2_float: Ultrasound sensor 2
    - 3_float: Ultrasound sensor 3
    - 4_float: Ultrasound sensor 4
    - 5_float: Ultrasound sensor 5
    - 6_float: Ultrasound sensor 6

- 5 -> Flexiforce sensors Data: Contains meassured values from four Flexiforce sensors.
  - sample: "<5 0.000000 0.000000 0.000000 0.000000"
  - code: 5
  - values:
    - 5: Flexiforce data code
    - 1_float: Flexiforce sensor 1
    - 2_float: Flexiforce sensor 2
    - 3_float: Flexiforce sensor 3
    - 4_float: Flexiforce sensor 4

- 6 -> Batteries sensors Data: Contains meassured values from four Batteries sensors. The amount of values depends fo the number of batteries and the number of cells of every battery.
  - sample: "<6 3 4 3.1 3.2 3.3 3.4 3.1 3.2 3.3 3.4 3.1 3.2 3.3 3.4"
  - code: 5
  - values:
    - 5: Flexiforce data code
    - 1_int: Number of batteries
    - 2_int: Number of cells for every battery
    - 3-n_float: Cell voltages.

- 7 -> Time Stamp Data: Current time in microseconds from Tiva board. Data is printed in two unsigned integers of 32 bits and need to be concatenated to get the real value that is a 64 bits unsigned integer.
  - sample: "<7 0 93852000"
  - code: 7
  - values:
    - 7: Time Stamp data code
    - 1_uint32: first 32 bits of time stamp
    - 1_uint32: last 32 bits of time stamp

- 8 -> Still Alive Message Data. A message that is sended form Tiva as a confirmation of non blocked device. Contains the current time in milliseconds.
  - sample: "<8 281263"
  - code: 8
  - values:
    - 8: Still Alive message data code
    - 1_uint32: time stamp

- 9 -> General message. This code indicates that message contains only command feedback data, useful in CLI mode.
  - sample: "<9 LED Auto Turn Off Enable"
  - code: 9
  - values:
    - 9: General message data code
    - string: message

- 10 -> Panel button status: Sends the status of panel button. After sending the message, the status will be cleared.
  - sample: "<10 2"
  - code: 10
  - values:
    - 10: Panel button data code
    - 1_int: button status. Posible values are:
      - RELEASED = 0
      - ONE_PRESSED = 1
      - ONE_LONG_PRESSED = 2
      - TWO_PRESSED = 3
      - THREE_PRESSED = 4
      - HARD_PRESSED = 5
      - NEVER_PRESSED = 6
      - FAILED_PRESSED = 7

- 12 -> Reseted board code. This code indicates that the Tiva board was reseted. Its configuration its now of default values.
  - sample: "<12 rt"
  - code: 12
  - values:
    - 12: Reseted board data code
    - string: command

- 13 -> Driver Status Message code. Sends the drivers status as current and half-bridges diagnostics. The lenght of this message can be diferent and depends of the number of motors and the motor model.
  - sample: "<13 2 4 0.0000 1 1 0.0000 1 1 0.0000 1 1 0.0000 1 1"
  - code: 13
  - values:
    - 13: Driver Status Message data code
    - 1_int: motor model. Posible values are:
      - NONE = 0
      - ESCON = 1
      - POLOLU = 2
    - 2_int: number of drivers
    - 3_float: current in driver 1
    - 4_float or 4_int: can be the current in driver 2 in Escon Drivers or half-bridge-a status for Pololu Drivers
    - 5_int: half-bridge-b status for Pololu Drivers
    - ...: all the remain data with the same previus pattern

## Command summary
Commands that you can send to board are:
- s: Read Status.
   - Syntax: 
      ```
      s
      ```
   - Args: none
   - Sample:
   ```
   s
   ```

- cf: Enable or disable command feedback, including prompt. All commands return a confirmation message, including configuration commands. Not applied to print commands.
   - Syntax: 
      ```
      cf [enab]
      ```
   - Args:
      - enab
         - 0: disable
         - 1: enable
   - Sample:
   ```
   cf 1
   ```

- sa: Enable or disable Still Alive messages. This message is printed periodically and is used to confirm that the board is connected and working correctly.
   - Syntax: 
      ```
      sa [enab]
      ```
   - Args:
      - enab
         - 0: disable
         - 1: enable
   - Sample:
   ```
   sa 1
   ```

- sk: *Acknowledge of Still Alive Message*. If sensors feedback and *Data Messages Timeout* are enable, this command reenable the sending of sensors message until timeout ocurs. This commnad is used as confirmation command from PC and avoids to send many commands from board that may overflow the serial buffer.
   - Syntax: 
      ```
      sk
      ```
   - Args: none
   - Sample:
   ```
   sk
   ```

- dt: Enable or disable *Data Messages Timeout*. This timeout stops the sending of sensor messages if *Acknowledge of Still Alive Message* is not arrived.
   - Syntax: 
      ```
      dt [enable]
      ```
   - Args:
      - enable
         - 0: disable
         - 1: enable
   - Sample:
   ```
   dt 1
   ```

- m: Move Motors. Enables or disables the motors and sets speeds.
   - Syntax:
      ```
      m
      m [enab] [motor1] [motor2]
      ```
   - Args:
      - enab: Enable or disable the motors. Only two values:
         - 0: disable
         - 1: enable
         - motor1: float value between 15 and 90, with sign. Set the PWM of motor 1 and direction of rotation.
         - motor2: float value between 15 and 90, with sign. Set the PWM of motor 2 and direction of rotation.
   - Sample:
   ```
   # Enable two motors
   m 1 -25 30

   # Stop motors
   m

   # Set speeds but not enable motors
   m 0 -45 40
   ```

- mt: Enable or disable Auto Stop Motor. Stop ocurs if not arrives a new motor command.
   - Syntax
      ```
      mt [enab]
      ```
   - Args:
      - enab:
         - 0: disable
         - 1: enable
   - Sample:
   ```
   mt 1
   ```

- c: Move Camera, using a servo motor.
   - Syntax:
      ```
      c [angle]
      ```
   - Args:
      - angle: Rotation of camera
   - Sample:
   ```
   c 45
   ```

- i: Read IMU and print its values.
   - Syntax:
      - i
   - Args: none
   - Sample:
   ```
   i
   ```

- if: Enable or disable automatic IMU Feedback messages.
   - Syntax:
      ```
      if [enab]
      ```
   - Args:
      - enab:
         - 0: disable
         - 1: enable
   - Sample:
   ```
   if 1
   ```

- o: Read and print Odometry values.
   - Syntax:
      ```
      o
      ```
   - Args: none
   - Sample:
   ```
   o
   ```

- of: Odom Feedback. Enable or disable automatic Odometry Feedback messages.
   - Syntax:
      ```
      of [enab]
      ```
   - Args:
      - enab:
         - 0: disable
         - 1: enable
   - Sample:
   ```
   of 1
   ```

- u: Read UltraSound
   - Syntax:
      ```
      u
      ```
   - Args: none
   - Sample:
   ```
   u
   ```

- uf: Ultrasound Feedback. Enable or disable automatic Ultrasound Sensors Feedback messages.
   - Syntax:
      ```
      uf [enab]
      ```
   - Args:
      - enab:
         - 0: disable
         - 1: enable
   - Sample:
   ```
   uf 1
   ```

- f: Read FlexiForce
   - Syntax:
      ```
      f
      ```
   - Args: none
   - Sample:
   ```
   f
   ```

- ff: Flefiforce Feedback. Enable or disable automatic Flexiforce Sensors Feedback messages.
   - Syntax:
      ```
      ff [enab]
      ```
   - Args:
      - enab:
         - 0: disable
         - 1: enable
   - Sample:
   ```
   ff 1
   ```

- b: Read Batteries
   - Syntax:
      ```
      b
      ```
   - Args: none
   - Sample:
   ```
   b
   ```

- bf: Batteries Feedback. Enable or disable automatic Monitor Batteries Feedback messages.
   - Syntax:
      ```
      bf [enab]
      ```
   - Args:
      - enab:
         - 0: disable
         - 1: enable
   - Sample:
   ```
   bf 1
   ```

- p: Read Panel Button
   - Syntax:
      ```
      p
      ```
   - Args: none
   - Sample:
   ```
   p
   ```

- l: Set RGB LED color in FlexiforceLEDButton_pcb.
   - Syntax:
      ```
      l [0-255] [0-255] [0-255]
      ```
   - Args:
     - R: 0 to 255
     - G: 0 to 255
     - B: 0 to 255
   - Sample:
   ```
   l 50 80 100
   ```

- n: Program a buzzer noise in FlexiforceLEDButton_PCB.
   - Syntax:
      ```
      n [0-255] [0-255] [0-255]
      ```
   - Args:
     - time_on: 0 to 255. How many millisecodns will be the buzzer in on mode
     - time_off: 0 to 255. How many millisecodns will be the buzzer in off mode
     - cicles: 0 to 255. Homw many times will be repeated the on-off cicle.
   - Sample:
   ```
   n 100 100 3
   ```

- sn: Program a buzzer noise using FlexiforceLEDButton_PCB and BatteryMonitor_PCBs.
   - Syntax:
      ```
      sn [0-255] [0-255] [0-255]
      ```
   - Args:
     - time_on: 0 to 255. How many millisecodns will be the buzzer in on mode
     - time_off: 0 to 255. How many millisecodns will be the buzzer in off mode
     - cicles: 0 to 255. Homw many times will be repeated the on-off cicle.
   - Sample:
   ```
   sn 100 100 3
   ```

- pf: Panel Button Feedback. Enable or disable automatic Pannel Button Feedback messages.
   - Syntax:
      ```
      pf [enab]
      ```
   - Args:
      - enab:
         - 0: disable
         - 1: enable
   - Sample:
   ```
   pf 1
   ```

- t: Read timeStamp
   - Syntax:
      ```
      p
      ```
   - Args: none
   - Sample:
   ```
   p
   ```

- rt: Reset Board.
   - Syntax:
      ```
      rt
      ```
   - Args: none
   - Sample:
   ```
   rt
   ```

- h: Print Help. Display list of commands.
   - Syntax:
      ```
      h
      ```
   - Args: none
   - Sample:
   ```
   h
   ```

- v: Show firmware version.
   - Syntax:
      ```
      v
      ```
   - Args: none
   - Sample:
   ```
   v
   ```

## Set automatic sending of data sensors

Use this command to configurate automatic sending of IMU sensor data
```
if 1
```

If you need that the board check if PC listening and sending confirmation messages to board, use this command:
```
dt 1
```

With previus command, automatic messages will stop if board dont receive a confirmation messages. Send the confirmation message with next command and see how automatic messages arrives again:
```
sk
```

## Version summary
- 2.4: Added Pololu and Escon motor classes.
- 2.3: Added command history in interactive mode. General code refactoring (Arduino consistent naming convention).
- 2.2: Added I2C scanner to avoid bus locking.
- 2.1: I2C initial implementation.
- 2.0: New initial version: Implements all older functionality in Platformio and separates pin definitions to be compatible with more boards.
- 1.0: Old Version, developed in Code Composer Studio. This project is based from that version.