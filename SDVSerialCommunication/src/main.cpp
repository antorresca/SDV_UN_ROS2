/**
 * SDV_Serial_Communication_Firmware
 * 
 * This sketch transforms a board (Tiva/MSP-EXP430F5529LP/Arduino) in a Serial 
 * Server that process commands comming through UART port and controls Escon 
 * Drivers of SDVUN1-3, read sensors and sends sensor data messages to NUC.
 * 
 * Values that this software can transmit are readed by "sdv_serial_node" or,
 * "agv_serial_node", a node that runs in ROS. Then, this values are transmited
 * in topics to Navigation Stack and other ROS software.
 * 
 * Author: Jhonyfer Angarita Moreno
 * Email: jhangaritamo@unal.edu.co
 */

#define FIRMWARE_VERSION "2.4.0"

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SparkFunMPU9250-DMP.h>
#include <SensorUpdater.h>
#include <EsconMotor.h>
#include <PololuMotor.h>
#include <TwoDriveController.h>
#include <FourDriveController.h>
#include <Battery.h>
#include <TimeWatcherTasks.h>
#include "tiva_pins.h"

//------------------------------------------------------------------------------
// Variables and defines for specific board
//
//------------------------------------------------------------------------------

#define SDV1        1
#define SDV2        2
#define SDV3        3
#define SDV4        4
#define SDV_MODEL   SDV4 // Change this to match with programing SDV

//------------------------------------------------------------------------------
// Unions, types, enums and structs
//------------------------------------------------------------------------------

// Contains a voltage in stored a float and in a byte array
union vflex
{
    float f;
    uint8_t b[4];
};

// Struct that stores Flexiforce data
struct FlexiforceA101
{
    vflex v_flex;
};

/*
Command line function callback type. Used to store functors in a table that can
called in a foor loop
*/
typedef int (*pfnCmdLine)(String arg);

// Command line function callback type.
typedef struct
{
    // A pointer to a string containing the name of the command.
    const char *pcCmd;

    // A function pointer to the implementation of the command.
    pfnCmdLine pfnCmd;

    // A pointer to a string of brief help text for the command.
    const char *psHelp;
} CMD_Struct;

// Button status.
enum Button_Status
{
    RELEASED = 0,
    ONE_PRESSED = 1,
    ONE_LONG_PRESSED = 2,
    TWO_PRESSED = 3,
    THREE_PRESSED = 4,
    HARD_PRESSED = 5,
    NEVER_PRESSED = 6,
    FAILED_PRESSED = 7,
};

// I2C Device Status
enum I2CDeviceStatus
{
    OK = 0,
    NOT_CONNECTED = 1
};

// I2C Device Struct
typedef struct
{
    uint8_t address;
    I2CDeviceStatus status;
    uint8_t i2c_port;
} I2CDevice;

//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------
void configMPU9250();
int setMotors(bool enable, double values[]);
void setI2cPort0();
void setI2cPort1();
void setI2cPort(I2CDevice device);
void setRGBColor(uint8_t r, uint8_t g, uint8_t b);
void stopMotors();
void turnOffRGBLED();
void setNoise(String args, bool all);
int setTimeWatcherFromCMD(String args, TimeWatcherTasks *time_watcher, String msg);
int processCommand();
void printFloat(float value, int places);
int countArguments(String args);
String getArgument(int index, String args);
void printHistoryCommand(bool upward);
String getCleanString(String s);
void cleanInputInSerial();
void checkTimeWatchers(unsigned long t);
void checkAutomaticMessages(unsigned long t);
void checkTime();
void checkI2cDevices(bool verbose);
void printStringAsAsciiBytes(String s);
void printStillAliveData();
void printImuData();
void printOdomData();
void printFakeImuData();
void printFlexiforceData();
void printButtonData();
void printUltrasoundData();
void printBatteriesData();
void printMotorDriversData();
void updateImuData();
void updateOdomData();
void updateFlexiforceData();
void updateButtonData();
void updateUltrasoundData();
void updateBatteriesData();
void updateMotorDriverSensor();
void updateMotorDriversData();
void emptyTask();
int enableSensorFeedback(String args, SensorUpdater &sensor_updater, String msg);
int readSensor(String args, SensorUpdater &sensor_updater);
int CMD_help(String args);
int CMD_moveMotor(String args);
int CMD_reset(String args);
int CMD_commandFeedback(String args);
int CMD_setCameraServo(String args);
int CMD_motorTimeout(String args);
int CMD_ledTimeout(String args);
int CMD_stillAliveMsg(String args);
int CMD_showVersion(String args);
int CMD_AknOfStillAliveMsg(String args);
int CMD_enabDataMsgTimeout(String args);
int CMD_setRGBColor(String args);
int CMD_setBuzzerNoise(String args);
int CMD_setSuperBuzzerNoise(String args);
int CMD_readTimestamp(String args);
int CMD_readImu(String args);
int CMD_readFlexiforce(String args);
int CMD_readOdometry(String args);
int CMD_readPanelButton(String args);
int CMD_readUltrasound(String args);
int CMD_readBatteries(String args);
int CMD_readDriversStatus(String args);
int CMD_imuFeedback(String args);
int CMD_flexiforceFeedback(String args);
int CMD_odometryFeedback(String args);
int CMD_buttonFeedback(String args);
int CMD_ultrasoundFeedback(String args);
int CMD_batteriesFeedback(String args);
int CMD_driversFeedback(String args);
int CMD_emptyFunction(String args);

//------------------------------------------------------------------------------
// Variables and constants
//------------------------------------------------------------------------------

// Variables: Board Status
bool reseted = true; // Fake Reseted Flag

// Variables: Commands
String input_string = "";                    // A String to hold incoming data
const uint32_t MAX_HISTORY = 10;
String input_history[MAX_HISTORY];           // Array that stores commands
uint8_t history_counter = 0;
uint8_t next_history_index = 0;
uint8_t show_history_index = 0;
bool received_empty_command = false;
char special_chars[5];                       // Array to store special characters
uint8_t special_chars_lenght = 0;            // Lenght of special characters buffer
bool special_char_received = false;
uint8_t wait_more_special_chars = 0;
bool enable_send_feedback_cmd = true;        // Control if some commands send confirmation messages.
uint8_t N_CMDS;                              // Number of commands in CMD_Table
bool stringComplete = false;                 // Whether the string is complete

// Variables: SA Message and Aknowledge Message
unsigned long last_alive_msg_time_stamp = 0; // Time stamp of last sended SA message
unsigned long last_akn_msg_time_stamp = 0;   // Time stamp of last sended aknowledge msg from NUC
bool enable_send_alive_msg = false;          // Flag to control sending of SA message
bool enable_sensor_data_timeout = false;     // Flag to timeout for sending Automatic Messages with high rate
unsigned long alive_msg_timeout = 1000;      // 1000 millisecs => 1Hz
unsigned long sensor_data_timeout = 1500;    // Timeout: if enable, disable sending IMU messages.

// Variables: Motors
#if SDV_MODEL == SDV1 || SDV_MODEL == SDV2 || SDV_MODEL == SDV3
// SDV1, SDV2 and SDV3 are difetential robots and only uses two motors
EsconMotor left_motor("left_m2", M2_PWM, M2_EN, M2_DIR, M2_SPEED_INPUT, M2_CURRENT_INPUT, true);
EsconMotor right_motor("right_m1", M1_PWM, M1_EN, M1_DIR, M1_SPEED_INPUT, M1_CURRENT_INPUT, false);
TwoDriveController motor_controller(&left_motor, &right_motor);
#endif

#if SDV_MODEL == SDV4
// SDV4 is a mecanum robot and uses four motors
PololuMotor back_left_motor("back_left_m2", P_M2_PWM, P_M2_DA, P_M2_DB, P_M2_ENC1, P_M2_ENC2, false);
PololuMotor back_right_motor("back_right_m1", P_M1_PWM, P_M1_DA, P_M1_DB, P_M1_ENC1, P_M1_ENC2, true);
PololuMotor front_left_motor("front_left_m3", P_M3_PWM, P_M3_DA, P_M3_DB, P_M3_ENC1, P_M3_ENC2, false);
PololuMotor front_right_motor("front_right_m4", P_M4_PWM, P_M4_DA, P_M4_DB, P_M4_ENC1, P_M4_ENC2, true);
FourDriveController motor_controller(&back_left_motor, &back_right_motor, &front_left_motor, &front_right_motor);
#endif

TimeWatcherTasks auto_stop_motor(emptyTask, stopMotors, 250);

// Variables: IMU
#if USE_IMU
MPU9250_DMP imu;
#endif
SensorUpdater imu_updater(updateImuData, printImuData, 50);

// Variables: Wheel Odometry
SensorUpdater odom_updater(updateOdomData, printOdomData, 50);

// Variables: Flexiforce
SensorUpdater flexiforce_updater(updateFlexiforceData, printFlexiforceData, 250);
FlexiforceA101 flex_01;
FlexiforceA101 flex_02;
FlexiforceA101 flex_03;
FlexiforceA101 flex_04;
FlexiforceA101 *flex_array[] = {&flex_01, &flex_02, &flex_03, &flex_04};

// Variables: Panel Button
SensorUpdater button_updater(updateButtonData, printButtonData, 1000);
Button_Status button_status = NEVER_PRESSED;

// Variables: Ultrasound sensors
SensorUpdater ultrasound_updater(updateUltrasoundData, printUltrasoundData, 500);

// Variables: Batteries voltage sensors
Battery battery_01;
Battery battery_02;
Battery battery_03;
Battery *batteries[] = {&battery_01, &battery_02, &battery_03};
SensorUpdater batteries_updater(updateBatteriesData, printBatteriesData, 1000);

// Variables: Motor Driver sensors
SensorUpdater motor_driver_updater(updateMotorDriversData, printMotorDriversData, 1000);

// Variables: CameraServo
#if SDV_MODEL == SDV4
Servo camera_servo;
#endif

// Variables: sensor tasks array
#define SENSOR_TASKS 7
SensorUpdater *sensor_updater_array[] =
{
    &imu_updater,
    &flexiforce_updater,
    &button_updater,
    &ultrasound_updater,
    &odom_updater,
    &batteries_updater,
    &motor_driver_updater,
};

// Variables: RGB LED
TimeWatcherTasks auto_stop_led(emptyTask, turnOffRGBLED, 1000);

// Variables: I2C Devices
#define IMU_ADDRESS                     0x68    // 0x68 -> 104
#define LAUNCHPAD_ADDRESS               0x5B    // 0x5B -> 91
#define ULTRASOUND_ADDRESS              0x5C    // 0x5C -> 92
#define MOTOR_DRIVER_ADDRESS            0x5D    // 0x5d -> 93
#define BATTERY_MONITOR_01_ADDRESS      0x51    // 0x51 -> 81
#define BATTERY_MONITOR_02_ADDRESS      0x52    // 0x52 -> 82
#define BATTERY_MONITOR_03_ADDRESS      0x53    // 0x53 -> 83
#define I2C_DEVICES                     7
#define I2C_BATTERIES_DEVICES           3
#define I2C_BUZZZER_DEVICES             4

I2CDevice imu_i2c_device = {IMU_ADDRESS, NOT_CONNECTED, 0};
I2CDevice flex_i2c_device = {LAUNCHPAD_ADDRESS, NOT_CONNECTED, 0};
I2CDevice ultrasound_i2c_device = {ULTRASOUND_ADDRESS, NOT_CONNECTED, 0};
I2CDevice motordrvr_i2c_device = {MOTOR_DRIVER_ADDRESS, NOT_CONNECTED, 0};
I2CDevice battery_01_i2c_device = {BATTERY_MONITOR_01_ADDRESS, NOT_CONNECTED, 0};
I2CDevice battery_02_i2c_device = {BATTERY_MONITOR_02_ADDRESS, NOT_CONNECTED, 0};
I2CDevice battery_03_i2c_device = {BATTERY_MONITOR_03_ADDRESS, NOT_CONNECTED, 0};
I2CDevice *i2c_devices[] = {
    &imu_i2c_device,
    &flex_i2c_device,
    &ultrasound_i2c_device,
    &battery_01_i2c_device,
    &battery_02_i2c_device,
    &battery_03_i2c_device,
    &motordrvr_i2c_device,
};
I2CDevice *i2c_batteries[] = {
    &battery_01_i2c_device,
    &battery_02_i2c_device,
    &battery_03_i2c_device,
};
I2CDevice *i2c_buzzers[] = {
    &flex_i2c_device,
    &battery_01_i2c_device,
    &battery_02_i2c_device,
    &battery_03_i2c_device,
};

// Time Watchers for special tasks
#define TIME_WATCHERS       2
TimeWatcherTasks *time_watchers[] = {
    &auto_stop_motor,
    &auto_stop_led,
};

// Strings
String enab = "Enable";
String disab = "Disable";
String newl_retcar = "\n\r";

#if SDV_MODEL == SDV1 || SDV_MODEL == SDV2 || SDV_MODEL == SDV3
const char move_motors_cmd[] = "-> Move Motors         Args: enable(0/1) ±DC_R ±DC_L";
#endif
#if SDV_MODEL == SDV4
const char move_motors_cmd[] = "-> Move Motors (rps)   Args: enable(0/1) ±b_left ±b_right ±f_left ±f_right";
#endif

// TimeStamp
union TimeStamp
{
    uint64_t microsecs_64;
    uint32_t microsecs_32[2];
};
TimeStamp time_stamp;

//------------------------------------------------------------------------------
// Table of valid command strings, callback functions and help messages.
//------------------------------------------------------------------------------
const CMD_Struct CMD_Table[] = {
    {"s",  CMD_emptyFunction,       "-> Read Status         Args: none"},
    {"cf", CMD_commandFeedback,     "-> Command feedback    Args: enable(0/1)"},
    {"sa", CMD_stillAliveMsg,       "-> Alive Timeout Msg   Args: enable(0/1)"},
    {"sk", CMD_AknOfStillAliveMsg,  "-> Acknowledge SAM     Args: none"},
    {"dt", CMD_enabDataMsgTimeout,  "-> Data Msg Timeout    Args: enable(0/1)"},
    {"m",  CMD_moveMotor,           move_motors_cmd},
    {"mt", CMD_motorTimeout,        "-> Auto Stop Motor     Args: enable(0/1)"},
    {"c",  CMD_setCameraServo,      "-> Move Camera         Args: pos_deg"},
    {"i",  CMD_readImu,             "-> Read IMU            Args: none"},
    {"if", CMD_imuFeedback,         "-> IMU Feedback        Args: enable(0/1)"},
    {"o",  CMD_readOdometry,        "-> Read Odometry       Args: none"},
    {"of", CMD_odometryFeedback,    "-> Odom Feedback       Args: enable(0/1)"},
    {"u",  CMD_readUltrasound,      "-> Read UltraSound     Args: none"},
    {"uf", CMD_ultrasoundFeedback,  "-> Ultras. Feedback    Args: enable(0/1)"},
    {"f",  CMD_readFlexiforce,      "-> Read FlexiForce     Args: none"},
    {"ff", CMD_flexiforceFeedback,  "-> Flexif. Feedback    Args: enable(0/1)"},
    {"b",  CMD_readBatteries,       "-> Read Batteries      Args: none"},
    {"bf", CMD_batteriesFeedback,   "-> Batteri. Feedback   Args: enable(0/1)"},
    {"p",  CMD_readPanelButton,     "-> Read Panel Button   Args: none"},
    {"pf", CMD_buttonFeedback,      "-> Button P. Feedback  Args: enable(0/1)"},
    {"d",  CMD_readDriversStatus,   "-> Show drivers data   Args: none"},
    {"df", CMD_driversFeedback,     "-> Drivers feedback    Args: enable(0/1)"},
    {"l",  CMD_setRGBColor,         "-> Set RGB color       Args: r(0-255) g(0-255) b(0-255)"},
    {"lt", CMD_ledTimeout,          "-> Auto turn-off LED   Args: enable(0/1)"},
    {"n",  CMD_setBuzzerNoise,      "-> Set Buzzer Noise    Args: on off cicles"},
    {"sn", CMD_setSuperBuzzerNoise, "-> Set S.Buzzer Noise  Args: on off cicles"},
    {"t",  CMD_readTimestamp,       "-> Read timeStamp      Args: none"},
    {"rt", CMD_reset,               "-> Reset Board         Args: none"},
    {"h",  CMD_help,                "-> Print Help          Display list of commands"},
    {"v",  CMD_showVersion,         "-> Version             Show firmware version"}};

//------------------------------------------------------------------------------
// CMD Line Status
//------------------------------------------------------------------------------
#define CMDLINE_CORRECT 0
#define CMDLINE_BAD_CMD (-1)
#define CMDLINE_TOO_MANY_ARGS (-2)
#define CMDLINE_TOO_FEW_ARGS (-3)
#define CMDLINE_INVALID_ARG (-4)

//------------------------------------------------------------------------------
// Setup
//------------------------------------------------------------------------------
void setup()
{
    // Serial configuration
    Serial.begin(BAUDRATE);
    Serial.setTimeout(1);
    Serial.println("\n\n\r\u001b[33mStarting...\u001b[0m");
    //while (!Serial);

    // Wait a litle moment, while every peripheral configurates their I2C port
    delay(1500);

    // Configure I2C physical port
    #if SDV_MODEL == SDV1 || SDV_MODEL == SDV2 || SDV_MODEL == SDV3
    imu_i2c_device.i2c_port = 1;
    #endif

    #if SDV_MODEL == SDV4
    for(uint8_t i = 0; i < I2C_DEVICES; i++)
    {
        (*i2c_devices[i]).i2c_port = 1;
    }
    #endif

    // I2C
    checkI2cDevices(true);

    // Start MPU9250
    #if USE_IMU
    if (imu_i2c_device.status == OK)
    {
        setI2cPort(imu_i2c_device);
        imu.begin();
        configMPU9250();
    }
    #endif

    // Configure Servo Camera
    #if SDV_MODEL == SDV4
    camera_servo.attach(SERVO_PIN);
    camera_servo.write(0);
    #endif

    // Attach MotorDriver Updater task to motor controller
    motor_controller.attachUpdateTask(updateMotorDriverSensor);
    motor_controller.stopMotors();

    // TO-DO: Check other I2C peripherals and generate SDV Status Code

    // Calculate number of commands in CMD_Table
    N_CMDS = sizeof(CMD_Table) / sizeof(CMD_Table[0]);
}

//------------------------------------------------------------------------------
// Loop
//------------------------------------------------------------------------------
void loop()
{
    // Check Time
    checkTime();

    // Check if Board is in fake reseted mode
    if (reseted)
    {
        enable_send_feedback_cmd = false;
        setRGBColor(0,0,0);

        reseted = false;
        for (uint8_t i = 0; i < TIME_WATCHERS; i++) (*time_watchers[i]).enableWatcher(false);
        for (uint8_t i = 0; i < SENSOR_TASKS; i++) (*sensor_updater_array[i]).enableSensorFeedback(false);
        enable_send_alive_msg = false;
        enable_sensor_data_timeout = false;
        enable_send_feedback_cmd = true;

        // Clean history
        for (uint32_t i = 0; i < MAX_HISTORY; i++)
        {
            input_history[i] = "";
        }
        next_history_index = 0;
        history_counter = 0;

        Serial.print("# ");
        Serial.print(time_stamp.microsecs_32[1]);
        Serial.print(" ");
        Serial.print(time_stamp.microsecs_32[0]);
        Serial.print(" ");
        Serial.print('F'); // SDV Status High Byte: Not used yet
        Serial.print(" ");
        Serial.print(37); // SDV Status Low Byte: Not used yet
        Serial.print(newl_retcar);
        Serial.print(">");
    }

    // Check if input message is complete. If true, proccess it
    if (stringComplete)
    {
        // Moves prompt to new line
        if (enable_send_feedback_cmd)
            Serial.print("\n\r");

        // Process received string as command
        int r_cmd = processCommand();

        // Store string in history array
        if(enable_send_feedback_cmd and !received_empty_command)
        {
            if(next_history_index >= MAX_HISTORY)
            {
                next_history_index = 0;
            }
            
            input_history[next_history_index] = input_string;
            next_history_index++;
            
            if(history_counter < MAX_HISTORY)
            {
                history_counter++;
            }
        }
        show_history_index = 0;
        

        // Clears input string variables
        stringComplete = false;
        input_string = "";

        // Print error in case of wrong command
        if (r_cmd != 1 and enable_send_feedback_cmd)
        {
            if (r_cmd == CMDLINE_BAD_CMD)
                Serial.print(F("Bad command!"));
            if (r_cmd == CMDLINE_TOO_MANY_ARGS)
                Serial.print(F("Too many arguments for command processor!"));
            if (r_cmd == CMDLINE_TOO_FEW_ARGS)
                Serial.print(F("Too few arguments for command processor!"));
            if (r_cmd == CMDLINE_INVALID_ARG)
                Serial.print(F("Invalid value in arguments"));

        }

        ////////////////////////////////////////////////////////////////////////
        // Make noise if a wrong command arrives
        /*
        if (
            r_cmd == CMDLINE_BAD_CMD || 
            r_cmd == CMDLINE_TOO_MANY_ARGS || 
            r_cmd == CMDLINE_TOO_FEW_ARGS ||
            r_cmd == CMDLINE_INVALID_ARG)
        {
            CMD_setBuzzerNoise("50 50 4");
        }
        */
        ////////////////////////////////////////////////////////////////////////

        // Print prompt
        if (enable_send_feedback_cmd)
        {
            if (received_empty_command)
            {
                received_empty_command = false;
            }
            else
            {
                Serial.print(newl_retcar);
            }
            Serial.print(">");
        }
    }

    // Time stamp for next actions:
    unsigned long t = millis();

    // Check for Time Watchers
    checkTimeWatchers(t);

    // Check for automatic data messages
    checkAutomaticMessages(t);

    // Update motor controller
    motor_controller.updateController(t);

    ////////////////////////////////////////////////////////////////////////////
    // Pin out to see Tiva cicle duration
    /*
    pinMode(PB_0, OUTPUT);
    digitalWrite(PB_0, !digitalRead(PB_0));
    */
    ////////////////////////////////////////////////////////////////////////////
}

void checkAutomaticMessages(unsigned long t)
{
    // Check if needs to send SA message
    if (enable_send_alive_msg)
    {
        if (t - last_alive_msg_time_stamp >= alive_msg_timeout)
        {
            printStillAliveData();
            last_alive_msg_time_stamp = t;
        }
    }

    // Check for Automatic Sensor Data Messages Timeout condition
    SensorUpdater::enableAllMessages(true);
    if (enable_sensor_data_timeout)
    {
        if ((t - last_akn_msg_time_stamp) > sensor_data_timeout)
        {
            SensorUpdater::enableAllMessages(false);
        }
    }

    /*
    Update time in SensorUpdater class. This method update sensors data or print its data
    if timeouts are reached.
    */
    SensorUpdater::updateTime();
    for (uint8_t i = 0; i < SENSOR_TASKS; i++)
    {
        (*sensor_updater_array[i]).updateTasK();
    }
}

/**
 * checkI2cDevices
 * 
 * Tries to connect to every listed I2C device and sets every status in an array
 * of integers.
 **/
void checkI2cDevices(bool verbose)
{
    if(USE_I2C == false)
        return;
        
    if(verbose)
        Serial.println(F("\u001b[33mScanning for I2C devices...\u001b[0m"));

    for (int i = 0; i < I2C_DEVICES; i++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        uint8_t address, error;
        address = (*i2c_devices[i]).address;
        setI2cPort((*i2c_devices[i]));
        Wire.beginTransmission(address);
        Wire.write(1);
        error = Wire.endTransmission(true);

        // Show status
        if(verbose)
        {
            if (error == 0)
            {
                (*i2c_devices[i]).status = OK;
                Serial.print(F("\u001b[36mFinded device in 0x"));
                Serial.print(address, HEX);
                Serial.print(F(" address "));
                Serial.println(F("\u001b[0m"));
            }
            else
            {
                (*i2c_devices[i]).status = NOT_CONNECTED;
                Serial.print(F("\u001b[31mError in 0x"));
                Serial.print(address, HEX);
                Serial.print(F(" address "));
                Serial.println(F("\u001b[0m"));
            }
        }
    }
}

void setI2cPort0()
{
    #if USE_TWO_I2C_PORTS
    Wire.setModule(0);
    #endif
    Wire.begin();
}

void setI2cPort1()
{
    #if USE_TWO_I2C_PORTS
    Wire.setModule(1);
    #endif
    Wire.begin();
}

void setI2cPort(I2CDevice device)
{
    if(device.i2c_port == 0) setI2cPort0();
    if(device.i2c_port == 1) setI2cPort1();
}

void setRGBColor(uint8_t r, uint8_t g, uint8_t b)
{
    // Write in I2C bus if device is connected
    if (flex_i2c_device.status == OK)
    {
        // Get command arguments
        uint8_t rgb[4];
        rgb[0] = 0x07;
        rgb[1] = r;
        rgb[2] = g;
        rgb[3] = b;

        // Set port
        setI2cPort(flex_i2c_device);

        // Write the command to Launchpad
        Wire.beginTransmission(flex_i2c_device.address);
        Wire.write(rgb, 4);
        Wire.endTransmission(true);
    }
}

void updateImuData()
{
    // IMU: Update data
    #if USE_IMU
    // Set port
    setI2cPort(imu_i2c_device);
    if (imu.fifoAvailable())
    {
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if (imu.dmpUpdateFifo() == INV_SUCCESS)
        {
            imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
        }
    }
    #endif
}

void updateOdomData()
{
    motor_controller.updateOdometry();
}

void updateFlexiforceData()
{
    // Check if device is connected
    if (flex_i2c_device.status == OK)
    {
        // Set port
        setI2cPort(flex_i2c_device);

        // Write a command to Launchap to change it's write mode
        Wire.beginTransmission(flex_i2c_device.address);
        Wire.write((uint8_t)0x05);
        Wire.endTransmission(true);
        delayMicroseconds(200);

        // Request data to Launchpad device
        Wire.requestFrom((uint8_t)flex_i2c_device.address, (uint8_t)16);

        // Reads I2C bus and stores bytes in an array
        uint8_t b_array[16];
        uint8_t index = 0;
        while (Wire.available())
        {                         // slave may send less than requested
            uint8_t c = Wire.read(); // receive a byte
            if (index < 16)
            {
                b_array[index] = c;
            }
            index++;
        }

        // Pass byte array to floats in every Flexiforce A101 instance
        for (uint8_t i = 0; i < 4; i++)
        {
            for (uint8_t j = 0; j < 4; j++)
            {
                (*flex_array[i]).v_flex.b[j] = b_array[i * 4 + j];
            }
        }
    }
}

void updateButtonData()
{
    if (flex_i2c_device.status == OK)
    {
        // Set port
        setI2cPort(flex_i2c_device);

        // Write a command to Launchap to change it's write mode
        Wire.beginTransmission(flex_i2c_device.address);
        Wire.write((uint8_t)0x06);
        Wire.endTransmission(true);
        delayMicroseconds(200);

        // Request data to Launchpad device
        Wire.requestFrom((uint8_t)flex_i2c_device.address, (uint8_t)1);

        // Reads I2C bus and stores bytes in an array
        byte data = 0;
        if (Wire.available())
        {
            data = Wire.read();
        }

        if (data == Button_Status::ONE_PRESSED or
            data == Button_Status::TWO_PRESSED or
            data == Button_Status::THREE_PRESSED or
            data == Button_Status::ONE_LONG_PRESSED or
            data == Button_Status::HARD_PRESSED or
            data == Button_Status::RELEASED)
        {
            button_status = (Button_Status)data;
        }
    }
}

void updateMotorDriverSensor()
{
    //Serial.print("Updating motor driver sensor\n\r");
    if (motordrvr_i2c_device.status == OK)
    {
        // Set port
        setI2cPort(motordrvr_i2c_device);

        // Command content
        uint8_t msg[4];
        msg[0] = 0x09; // Command id
        msg[1] = 0x01; // Enable half-bridges

        // Write a command to motor driver sensor to enable A and B half-bridges
        Wire.beginTransmission(motordrvr_i2c_device.address);
        Wire.write(msg, 2);
        Wire.endTransmission(true);
        delayMicroseconds(200);
    }
}

void updateUltrasoundData()
{
    // TODO
}

void updateBatteriesData()
{
    uint8_t data_length = 16;
    uint8_t buffer_array[data_length];

    // Iteration over batteries array
    for(uint8_t i = 0; i < I2C_BATTERIES_DEVICES; i++)
    {
        // Clear buffer data
        for (uint8_t j = 0; j < 16; j++)
            buffer_array[j] = 0;
        
        // Check if device is OK.
        if(i2c_batteries[i]->status == OK)
        {
            // Request data from device
            Wire.requestFrom(i2c_batteries[i]->address, data_length);

            // Read arrived data if available
            uint8_t index = 0;
            while (Wire.available())
            {
                uint8_t c = Wire.read();
                if (index < 16)
                {
                    buffer_array[index] = c;
                }
                index++;
            }

            // Store data in battery object
            batteries[i]->buffer2vcells(buffer_array);
        }
    }
}

void updateMotorDriversData()
{
    if (motordrvr_i2c_device.status == OK)
    {
        // Set port
        setI2cPort(motordrvr_i2c_device);

        // Request data to Launchpad device
        Wire.requestFrom((uint8_t)motordrvr_i2c_device.address, (uint8_t)24);

        // Reads I2C bus and stores bytes in an array
        uint8_t data[24];
        uint8_t index = 0;
        while (Wire.available())
        {
            if (index < 24) 
                data[index] = Wire.read();
            index++;
        }

        // Set data in motor controller
        motor_controller.setMotorStatus(data);
    }
}

void configMPU9250()
{
    #if USE_IMU
    // Set port
    setI2cPort(imu_i2c_device);

    // Enable all sensors, and set sample rates to 4Hz.
    // (Slow so we can see the interrupt work.)
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    imu.setSampleRate(10);        // Set accel/gyro sample rate (Hz)
    imu.setCompassSampleRate(10); // Set mag rate (Hz)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(42); // Set LPF corner frequency to 5Hz

    // Use enableInterrupt() to configure the MPU-9250's
    // interrupt output as a "data ready" indicator.
    //imu.enableInterrupt();

    // The interrupt level can either be active-high or low.
    // Configure as active-low, since we'll be using the pin's
    // internal pull-up resistor.
    // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
    imu.setIntLevel(INT_ACTIVE_LOW);

    // The interrupt can be set to latch until data has
    // been read, or to work as a 50us pulse.
    // Use latching method -- we'll read from the sensor
    // as soon as we see the pin go LOW.
    // Options are INT_LATCHED or INT_50US_PULSE
    imu.setIntLatched(INT_LATCHED);

    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(2000); // Full Scale Range

    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(2); // Set accel to +/-2g

    // Using the Digital Motion Processor
    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |  // Enable 6-axis quat
                     DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 20);                      // Set DMP FIFO rate to 20 Hz
    #endif
}

void checkTimeWatchers(unsigned long time)
{
    for(uint8_t i = 0; i < TIME_WATCHERS; i++)
        time_watchers[i]->check(time);
}

void stopMotors()
{
    motor_controller.stopMotors();

    // Debug is my passion!!!
    //motor_controller.printMotorsStatus();
    //Serial.print("\n\r");
}

void turnOffRGBLED()
{
    setRGBColor(0, 0, 0);
}

int setMotors(bool enable, double values[])
{

    // Enable argument cant stop the motors if it's false
    if (enable == false)
    {
        stopMotors();
        return 0;
    }

    //double values[] = {duty_cycle_L, duty_cycle_R};
    int r = motor_controller.setSpeeds(enable, values);

    // Debug is my passion!!!
    //motor_controller.printMotorsStatus();
    //Serial.print("\n\r");

    return r;
}


void setNoise(String args, bool all)
{
    // Get Buzzer Noise arguments
    uint8_t noise[3];
    noise[0] = getArgument(0, args).toInt();
    noise[1] = getArgument(1, args).toInt();
    noise[2] = getArgument(2, args).toInt();

    // If all flag is true, send noise command to all buzzer devices
    // else, send noise command only to main buzzer
    uint8_t buzzer_devices = I2C_BUZZZER_DEVICES;
    if(!all)
    {
        buzzer_devices = 1;
    }

    uint8_t noise_commands[] = {0x08, 0x02, 0x02, 0x02};
    for(uint8_t i = 0; i < buzzer_devices; i++)
    {
        // Write in I2C bus if device is connected
        if (i2c_buzzers[i]->status == OK)
        {
            // Set port
            setI2cPort(*i2c_buzzers[i]);

            // Write the command to Launchpad
            Wire.beginTransmission(i2c_buzzers[i]->address);
            Wire.write(noise_commands[i]);
            Wire.write(noise, 3);
            Wire.endTransmission(true);
        }
    }

    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        Serial.print(F("<9 Setted Buzzer Noise values: On = "));
        Serial.print(noise[0]);
        Serial.print(F(", Off = "));
        Serial.print(noise[1]);
        Serial.print(F(", Cicles = "));
        Serial.print(noise[2]);
        Serial.print(F(", All = "));
        Serial.print(all);
    }
}

/**
 * serialEvent
 * 
 * This method is called when a byte is writen in Serial
 * input. Reads the byte and stores it in a string. Then, resends the byte,
 * in Serial output for feedback, like a terminal.
 */
void serialEvent()
{
    while (Serial.available())
    {
        // Get the new byte
        char received_char = (char)Serial.read();

        // If it's waiting or more special chars, add received char and exit
        if (special_char_received and wait_more_special_chars > 0)
        {
            special_chars[special_chars_lenght] = received_char;
            special_chars_lenght++;
            wait_more_special_chars--;

            // Received all special chars: processing it
            if(wait_more_special_chars == 0)
            {
                special_char_received = false;
                if(special_chars[0] == 27)
                {
                    if(special_chars[1] == '[')
                    {
                        // Up Arrow
                        if(special_chars[2] == 'A' and enable_send_feedback_cmd)
                        {
                            cleanInputInSerial();
                            printHistoryCommand(true);
                        }
                        // Down Arrow
                        if(special_chars[2] == 'B' and enable_send_feedback_cmd)
                        {
                            cleanInputInSerial();
                            printHistoryCommand(false);
                        }
                    }
                }
            }
            return;
        }
        
        // Add received char to the input_string if isn't a special character
        if (received_char >= 32 and received_char <= 126)
        {
            input_string += received_char;
            if (enable_send_feedback_cmd)
            {
                Serial.write(received_char);
            }
        }
        else
        {
            // ESC char received: can contain a arrow command
            if (received_char == 27)
            {
                special_chars[0] = received_char;
                special_chars_lenght = 1;
                special_char_received = true;
                wait_more_special_chars = 2;
            }

            /*
            if the incoming character is a Car Return, set a flag so the main 
            loop can do something about it
            */
            if (received_char == '\r')
            {
                input_string += received_char; // Adds '\r'
                input_string += '\n';
                stringComplete = true;
            }

            // Delete char from screen and the string if char is 127 (DEL in ASCII)
            if (received_char == 127)
            {
                if (input_string.length() >= 1)
                {
                    if (enable_send_feedback_cmd)
                    {
                        Serial.write(0x08);
                        Serial.write(' ');
                        Serial.write(0x08);
                    }
                    input_string.remove(input_string.length() - 1);
                }
            }
        }
    }
}

void printHistoryCommand(bool upward)
{
    // At least one command is stored in history
    if(history_counter > 0)
    {
        // Go to up history element
        if(upward)
        {
            if(show_history_index < history_counter)
            {
                show_history_index++;
            }
        }
        
        // Go to down history element
        if(!upward)
        {
            if(show_history_index > 0)
            {
                show_history_index--;
            }
        }

        // Out of history elements: clear input
        if(show_history_index == 0)
        {
            input_string = "";
            return;
        }

        // Obtains absolute index in hisotry
        int8_t absolute_history_index = 0;
        if(next_history_index < history_counter)
        {
            absolute_history_index = next_history_index - show_history_index;
            if(absolute_history_index < 0)
            {
                absolute_history_index += MAX_HISTORY;
            }
        }
        if(next_history_index == history_counter)
        {
            absolute_history_index = history_counter - show_history_index;
        }

        // Get string from history and print it
        input_string = getCleanString(input_history[absolute_history_index]);
        Serial.print(input_string);
    }
}

void cleanInputInSerial()
{
    unsigned int l = input_string.length();
    for(unsigned int i = 0; i < l; i++)
    {
        Serial.write(0x08);
        Serial.write(' ');
        Serial.write(0x08);
    }
    input_string = "";
}

void printFloat(float value, int places)
{
    // this is used to cast digits
    int digit;
    float tens = 0.1;
    int tenscount = 0;
    int i;
    float tempfloat = value;

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
    // if this rounding step isn't here, the value  54.321 prints as 54.3209

    // calculate rounding term d:   0.5/pow(10,places)
    float d = 0.5;
    if (value < 0)
        d *= -1.0;
    // divide by ten for each decimal place
    for (i = 0; i < places; i++)
        d /= 10.0;
    // this small addition, combined with truncation will round our values properly
    tempfloat += d;

    // first get value tens to be the large power of ten less than value
    // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

    if (value < 0)
        tempfloat *= -1.0;
    while ((tens * 10.0) <= tempfloat)
    {
        tens *= 10.0;
        tenscount += 1;
    }

    // write out the negative if needed
    if (value < 0)
        Serial.print('-');

    if (tenscount == 0)
        Serial.print(0, DEC);

    for (i = 0; i < tenscount; i++)
    {
        digit = (int)(tempfloat / tens);
        Serial.print(digit, DEC);
        tempfloat = tempfloat - ((float)digit * tens);
        tens /= 10.0;
    }

    // if no places after decimal, stop now and return
    if (places <= 0)
        return;

    // otherwise, write the point and continue on
    Serial.print('.');

    // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
    for (i = 0; i < places; i++)
    {
        tempfloat *= 10.0;
        digit = (int)tempfloat;
        Serial.print(digit, DEC);
        // once written, subtract off that digit
        tempfloat = tempfloat - (float)digit;
    }
}

/**
 * process_CMD
 * 
 * Process inputString and find commands contained in it. Then, search in CMD 
 * table and send string arguments to command.
 */
int processCommand()
{
    // Verifies size of input string
    int s = input_string.length();
    if (s < 1)
    {
        return CMDLINE_CORRECT;
    }

    // Find initial index location of command in string
    int i_index = 0;
    for (int i = 0; i < s; i++)
    {
        i_index = i;
        if (input_string.charAt(i) != ' ')
            break;
    }

    // Find final index location of command in string
    int f_index = 0;
    for (int i = i_index; i < s; i++)
    {
        f_index = i;
        if (input_string.charAt(i) == ' ' or input_string.charAt(i) == '\r')
            break;
    }

    // Get command string using initial and final indexes
    String cmd = input_string.substring(i_index, f_index);

    // Compares with CMD_Table
    for (uint8_t i = 0; i < N_CMDS; i++)
    {
        String cmd_t = String(CMD_Table[i].pcCmd);
        if (cmd.equals(cmd_t))
        {
            return CMD_Table[i].pfnCmd(input_string.substring(f_index, s));
        }
    }

    // DEBUG
    //printStringAsAsciiBytes(input_string);

    // Command is empty
    if (countArguments(input_string) == 0)
    {
        received_empty_command = true;
        return CMDLINE_CORRECT;
    }

    // Received command is bad
    return CMDLINE_BAD_CMD;
}

/**
 * count_arguments
 *
 * Calculate the number of arguments contained in received string.
 */
int countArguments(String args)
{
    int counter = 0;
    char prev_char = ' ';
    for (unsigned int i = 0; i < args.length(); i++)
    {
        char act_char = args.charAt(i);
        if (prev_char == ' ' and act_char != ' ' and act_char != '\n' and act_char != '\r')
            counter++;
        prev_char = act_char;
    }
    return counter;
}

/*
 * get_argument
 *
 * Returns the n argument contained in received string.
 */
String getArgument(int index, String args)
{
    int counter = 0;
    char prev_char = ' ';
    String a = "";
    for (unsigned int i = 0; i < args.length(); i++)
    {
        char act_char = args.charAt(i);
        if (prev_char == ' ' and act_char != ' ' and act_char != '\n' and act_char != '\r')
            counter++;
        prev_char = act_char;

        if (index == (counter - 1) and act_char != ' ' and act_char != '\n' and act_char != '\r')
            a += act_char; // Ading char to string
    }
    return a;
}

String getCleanString(String s)
{
    String clean = "";
    for (unsigned int i = 0; i < s.length(); i++)
    {
        char act_char = s.charAt(i);
        if (act_char != '\n' and act_char != '\r' and act_char != 27)
            clean += act_char;
    }
    return clean;
}

void printStringAsAsciiBytes(String s)
{
    for (uint32_t i = 0; i < s.length(); i++)
    {
        uint8_t c = s.charAt(i);
        Serial.print(F("Char "));
        Serial.print(i);
        Serial.print(F(" = "));
        Serial.print(c);
        Serial.print(F("\n\r"));
    }
}

void print_timestamp_data()
{
    // Print TimeStamp in two unsigned integers of 32 bits length
    Serial.print(F("<7 "));
    Serial.print(time_stamp.microsecs_32[1]);
    Serial.print(" ");
    Serial.print(time_stamp.microsecs_32[0]);
}

void printStillAliveData()
{
    Serial.print("<8 ");
    Serial.print(millis());
    Serial.print(newl_retcar);
    Serial.flush();
}

void printImuData()
{
    // Check if IMU is connected and print its data
    if (imu_i2c_device.status == OK)
    {
        #if USE_IMU
        float accelX = imu.calcAccel(imu.ax);
        float accelY = imu.calcAccel(imu.ay);
        float accelZ = imu.calcAccel(imu.az);
        float gyroX = imu.calcGyro(imu.gx);
        float gyroY = imu.calcGyro(imu.gy);
        float gyroZ = imu.calcGyro(imu.gz);
        float magX = imu.calcMag(imu.mx);
        float magY = imu.calcMag(imu.my);
        float magZ = imu.calcMag(imu.mz);

        imu.computeEulerAngles();
        float qX = imu.calcQuat(imu.qx);
        float qY = imu.calcQuat(imu.qy);
        float qZ = imu.calcQuat(imu.qz);
        float qW = imu.calcQuat(imu.qw);

        Serial.print("<1 ");

        printFloat(accelX, 6);
        Serial.print(" ");
        printFloat(accelY, 6);
        Serial.print(" ");
        printFloat(accelZ, 6);
        Serial.print(" ");

        printFloat(gyroX, 6);
        Serial.print(" ");
        printFloat(gyroY, 6);
        Serial.print(" ");
        printFloat(gyroZ, 6);
        Serial.print(" ");

        printFloat(magX, 6);
        Serial.print(" ");
        printFloat(magY, 6);
        Serial.print(" ");
        printFloat(magZ, 6);
        Serial.print(" ");

        printFloat(qX, 6);
        Serial.print(" ");
        printFloat(qY, 6);
        Serial.print(" ");
        printFloat(qZ, 6);
        Serial.print(" ");
        printFloat(qW, 6);
        #else
        printFakeImuData();
        #endif
    }
    else // IMU disconnected, print fake data
    {
        printFakeImuData();
    }
}

void printFakeImuData()
{
    // Print fake data
    Serial.print("<1 ");
    for (int i = 0; i < 12; i++)
    {
        Serial.print(0.0);
        Serial.print(" ");
    }
    Serial.print(1.0);
    Serial.print(" ");
}

void printOdomData()
{
    Serial.print("<2 ");
    motor_controller.printSpeeds();
}

void printFlexiforceData()
{
    Serial.print("<5 ");
    for (uint8_t i = 0; i < 4; i++)
    {
        Serial.print((*flex_array[i]).v_flex.f, 3);
        Serial.print(" ");
    }
}

void printButtonData()
{
    Serial.print("<10 ");
    Serial.print(button_status);
}

void printUltrasoundData()
{
    // TODO
    // Fake data for ultrasound sensors
    Serial.print("<4 ");
    for(uint8_t i = 0; i < 6; i++)
    {
        printFloat(0.0, 6);
        Serial.print(" ");
    }
}

void printBatteriesData()
{
    Serial.print("<6 ");
    Serial.print(I2C_BATTERIES_DEVICES);  //Amount of batteries
    Serial.print(" ");
    Serial.print(4);  // Number of cells per battery
    Serial.print(" ");
    
    // Iteration over batteries array
    for(uint8_t i = 0; i < I2C_BATTERIES_DEVICES; i++)
    {
        batteries[i]->print_vcells();
        Serial.print(" ");
    }
}

void printMotorDriversData()
{
    Serial.print("<13 ");
    Serial.print(motor_controller.getMotorModel());
    Serial.print(" ");
    Serial.print(motor_controller.n_motors);
    Serial.print(" ");
    motor_controller.printMotorsStatus();
}

void checkTime()
{
    // Converting millis to micros. Previus solution fails, due to precision
    // issues in micros() function in Tiva/Energia Boards.

    // For compatibility with "serial_agv_node", time stamp is reported in two
    // integers of 32 bits every one. microsecs long integer has to be splited.

    // Using a Union to store millis converted to micros, without presicion of
    // micros function.
    time_stamp.microsecs_64 = ((uint64_t)millis()) * 1000;
}

int enableSensorFeedback(String args, SensorUpdater &sensor_updater, String msg)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 1)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 1)
        return CMDLINE_TOO_FEW_ARGS;

    // Get first argument
    sensor_updater.enableSensorFeedback(getArgument(0, args).toInt());

    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        if (sensor_updater.isSensorFeedbackEnabled())
        {
            Serial.print(msg + enab);
        }
        else
        {
            Serial.print(msg + disab);
        }
    }

    // Exit
    return CMDLINE_CORRECT;
}

int readSensor(String args, SensorUpdater &sensor_updater)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 0)
        return CMDLINE_TOO_MANY_ARGS;

    // Update sensor data
    sensor_updater.updateData();

    // Print data
    sensor_updater.printData();

    // Exit
    return CMDLINE_CORRECT;
}

int setTimeWatcherFromCMD(String args, TimeWatcherTasks *time_watcher, String msg)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 1)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 1)
        return CMDLINE_TOO_FEW_ARGS;

    // Set Auto Stop Motors flag
    bool enable_watcher = (bool)getArgument(0, args).toInt();
    time_watcher->enableWatcher(enable_watcher);

    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        enable_watcher ? Serial.print(msg + enab) : Serial.print(msg + disab);
    }

    return CMDLINE_CORRECT;
}

void emptyTask(){}

int CMD_help(String args)
{
    Serial.print(newl_retcar);
    Serial.print(F("\n\rAvailable Commands\n\r------------------\n\n\r"));

    for (uint8_t i = 0; i < N_CMDS; i++)
    {
        Serial.print(CMD_Table[i].pcCmd);                           // Print char command
        for (uint8_t j = 0; j < (3 - strlen(CMD_Table[i].pcCmd)); j++) // Adding spaces
            Serial.print(" ");
        Serial.print(CMD_Table[i].psHelp); // Print Help content
        Serial.print(newl_retcar);         // Go to Next line in screen
    }
    return CMDLINE_CORRECT;
}

int CMD_showVersion(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 0)
        return CMDLINE_TOO_MANY_ARGS;

    // Print firmware version
    Serial.print(F("<9 SDVUN-FIRMWARE_"));
    Serial.print(FIRMWARE_VERSION);
    Serial.print(" for SDV");
    Serial.print(SDV_MODEL);
    Serial.print(" mobile robot");
    return CMDLINE_CORRECT;
}

int CMD_moveMotor(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    
    // 1 Argument: Quick stop
    if (count == 0)
    {
        // Stopping motors
        stopMotors();

        // Updating motor time stamp
        auto_stop_motor.feed(millis());

        // Print feddback command
        if (enable_send_feedback_cmd)
            Serial.print(F("<0 m"));

        // Exit
        return CMDLINE_CORRECT;
    }
    if (count > motor_controller.n_motors + 1)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < motor_controller.n_motors + 1)
        return CMDLINE_TOO_FEW_ARGS;

    // Get first argument
    bool enable = (bool)getArgument(0, args).toInt();

    //Get other arguments
    double values[] = {0,0,0,0};
    for(uint8_t i = 0; i < motor_controller.n_motors; i++)
    {
        values[i] = getArgument(1 + i, args).toFloat();
    }

    // Configuring Motors
    int result = setMotors(enable, values);
    if (result == -1)
        return CMDLINE_INVALID_ARG;

    // Updating motor time stamp
    auto_stop_motor.feed(millis());

    // Print feddback command
    if (enable_send_feedback_cmd)
        Serial.print(F("<0 m"));

    return CMDLINE_CORRECT;
}

int CMD_reset(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 1)
        return CMDLINE_TOO_MANY_ARGS;

    // Dummy reset
    stopMotors();
    Serial.print(F("<12 rt"));
    reseted = true;

    return CMDLINE_CORRECT;
}

int CMD_readTimestamp(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 0)
        return CMDLINE_TOO_MANY_ARGS;

    // Check Time
    checkTime();

    // Print timestamp
    print_timestamp_data();

    return CMDLINE_CORRECT;
}

int CMD_motorTimeout(String args)
{
   String ms = F("<9 Motor Auto Stop ");
   return setTimeWatcherFromCMD(args, &auto_stop_motor, ms);
}

int CMD_ledTimeout(String args)
{
   String ms = F("<9 LED Auto Turn Off ");
   return setTimeWatcherFromCMD(args, &auto_stop_led, ms); 
}

int CMD_commandFeedback(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 1)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 1)
        return CMDLINE_TOO_FEW_ARGS;

    // Get first argument
    enable_send_feedback_cmd = (bool)getArgument(0, args).toInt();

    if (enable_send_feedback_cmd)
        Serial.print(F("<9 Command Feedback enable"));

    return CMDLINE_CORRECT;
}

int CMD_stillAliveMsg(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 1)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 1)
        return CMDLINE_TOO_FEW_ARGS;

    // Get first argument
    enable_send_alive_msg = (bool)getArgument(0, args).toInt();

    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        String ms = F("<9 Send of Still Alive Messages ");
        if (enable_send_alive_msg)
        {
            Serial.print(ms + enab);
        }
        else
        {
            Serial.print(ms + disab);
        }
    }

    // Exit
    return CMDLINE_CORRECT;
}

int CMD_AknOfStillAliveMsg(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 0)
        return CMDLINE_TOO_MANY_ARGS;

    // Store time stamp of aknowledge message
    last_akn_msg_time_stamp = millis();

    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        String ms = F("<9 Aknowledge of Still Alive Message Received");
        Serial.print(ms);
    }

    // Exit
    return CMDLINE_CORRECT;
}

int CMD_enabDataMsgTimeout(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 1)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 1)
        return CMDLINE_TOO_FEW_ARGS;

    // Get first argument
    enable_sensor_data_timeout = (bool)getArgument(0, args).toInt();

    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        String ms = F("<9 Data Sensor Messages Timeout ");
        if (enable_sensor_data_timeout)
        {
            Serial.print(ms + enab);
        }
        else
        {
            Serial.print(ms + disab);
        }
    }

    // Exit
    return CMDLINE_CORRECT;
}

int CMD_setRGBColor(String args)
{
    // Get number of arguments
    int count = countArguments(args);

    // Quick reset: 0 arguments
    if (count == 0)
    {
        turnOffRGBLED();
        if(enable_send_feedback_cmd) 
            Serial.print(F("<9 Setted RGB values: R = 0, G = 0, B = 0"));
        return CMDLINE_CORRECT;
    }

    // Set color using 3 arguments: check number of arrived args
    if (count > 3)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 3)
        return CMDLINE_TOO_FEW_ARGS;

    // Send values of RGB LED
    int r = getArgument(0, args).toInt();
    int g = getArgument(1, args).toInt();
    int b = getArgument(2, args).toInt();
    setRGBColor(r, g, b);

    // Feed the watcher
    auto_stop_led.feed(millis());
    
    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        Serial.print(F("<9 Setted RGB values: R = "));
        Serial.print(r);
        Serial.print(F(", G = "));
        Serial.print(g);
        Serial.print(F(", B = "));
        Serial.print(b);
    }

    // Exit
    return CMDLINE_CORRECT;
}

int CMD_setBuzzerNoise(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 3)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 3)
        return CMDLINE_TOO_FEW_ARGS;

    // Send command noise to main buzzer
    setNoise(args, false);

    // Exit
    return CMDLINE_CORRECT;
}

int CMD_setSuperBuzzerNoise(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 3)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 3)
        return CMDLINE_TOO_FEW_ARGS;

    // Send command noise to all buzzers
    setNoise(args, true);

    // Exit
    return CMDLINE_CORRECT;
}

int CMD_setCameraServo(String args)
{
    // Verifies that command contains correct amount of arguments
    int count = countArguments(args);
    if (count > 1)
        return CMDLINE_TOO_MANY_ARGS;
    if (count < 1)
        return CMDLINE_TOO_FEW_ARGS;

    // Get deg value and check if is correct
    int deg = getArgument(0, args).toInt();
    if(deg < 0 || deg > 180)
    {
        return CMDLINE_INVALID_ARG;
    }

    // Set Servo pose
    #if SDV_MODEL == SDV4
    camera_servo.write(deg);
    #endif

    // Print feedback message
    if (enable_send_feedback_cmd)
    {
        String ms = F("<9 Servo Camera pose changed to ");
        Serial.print(ms);
        Serial.print(deg);
    }

    // Exit
    return CMDLINE_CORRECT;
}

int CMD_readImu(String args)
{
    return readSensor(args, imu_updater);
}

int CMD_readOdometry(String args)
{
    return readSensor(args, odom_updater);
}

int CMD_readFlexiforce(String args)
{
    return readSensor(args, flexiforce_updater);
}

int CMD_readPanelButton(String args)
{
    return readSensor(args, button_updater);
}

int CMD_readUltrasound(String args)
{
    return readSensor(args, ultrasound_updater);
}

int CMD_readBatteries(String args)
{
    return readSensor(args, batteries_updater);
}

int CMD_readDriversStatus(String args)
{
    return readSensor(args, motor_driver_updater);
}

int CMD_imuFeedback(String args)
{
    return enableSensorFeedback(args, imu_updater, F("<9 IMU Feedback "));
}

int CMD_odometryFeedback(String args)
{
    return enableSensorFeedback(args, odom_updater, F("<9 Odometry Feedback "));
}

int CMD_flexiforceFeedback(String args)
{
    return enableSensorFeedback(args, flexiforce_updater, F("<9 Flexiforce Feedback "));
}

int CMD_buttonFeedback(String args)
{
    return enableSensorFeedback(args, button_updater, F("<9 Panel Button Feedback "));
}

int CMD_driversFeedback(String args)
{
    return enableSensorFeedback(args, motor_driver_updater, F("<9 Motor Driver Feedback "));
}

int CMD_batteriesFeedback(String args)
{
    return enableSensorFeedback(args, batteries_updater, F("<9 Batteries voltages Feedback "));
}

int CMD_ultrasoundFeedback(String args)
{
    return enableSensorFeedback(args, ultrasound_updater, F("<9 Ultrasound Sensors Feedback "));
}

int CMD_emptyFunction(String args)
{
    if (enable_send_feedback_cmd)
    {
        Serial.print(F("TO-DO"));
    }
    return CMDLINE_CORRECT;
}
