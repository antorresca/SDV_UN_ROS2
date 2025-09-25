/**
 * tiva_pins.h
 * 
 * Defines for TI Tiva C Series EK-TM4C123GXL Board.
 * 
 * Author: Jhonyfer Angarita Moreno
 * Email: jhangaritamo@unal.edu.co
 */

// Pin Defines

// Escon Motor pins
#define M1_DIR             PF_1
#define M1_EN              PF_2
#define M1_PWM             PB_7
#define M1_SPEED_INPUT     PC_7
#define M1_CURRENT_INPUT   PC_5
#define M2_DIR             PF_3
#define M2_EN              PF_4
#define M2_PWM             PB_6
#define M2_SPEED_INPUT     PC_6
#define M2_CURRENT_INPUT   PC_4

// Pololu Motor pins
#define P_M1_PWM           PF_2
#define P_M1_DA            PD_0
#define P_M1_DB            PD_1
#define P_M1_ENC1          PC_6
#define P_M1_ENC2          PC_5

#define P_M2_PWM           PF_3
#define P_M2_DA            PD_2
#define P_M2_DB            PD_3
#define P_M2_ENC1          PC_7
#define P_M2_ENC2          PD_6

#define P_M3_PWM           PB_3
#define P_M3_DA            PE_1
#define P_M3_DB            PE_2
#define P_M3_ENC1          PD_7
#define P_M3_ENC2          PF_4

#define P_M4_PWM           PC_4
#define P_M4_DA            PE_5
#define P_M4_DB            PA_4
#define P_M4_ENC1          PA_2
#define P_M4_ENC2          PE_4

// I2C pins
#define SDA                PA_7
#define SCL                PA_6
#define ANALOG_REFERENCE   3.3

// Camera Servo
#define SERVO_PIN          PB_2

// Tiva overlaps RGB leds with motor pins. Don't use Tiva built in LEDs
#define RED_LED            PF_1
#define GREEN_LED          PF_3
#define BLUE_LED           PF_2

// Baudrate
#define BAUDRATE           921600

// I2C Module configuration
#define USE_I2C            true
#define USE_IMU            false
#define USE_TWO_I2C_PORTS  false
