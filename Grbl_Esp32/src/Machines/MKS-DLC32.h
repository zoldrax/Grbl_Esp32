#pragma once
// clang-format off

/*
    Pinout for the MKS DLC32 Board for grbl ESP32
    https://github.com/makerbase-mks/MKS-DLC32

    Cheap 3 Axis board with Laser out and SD

    - 3 independent Stepper Axis via Stepstick drivers
    - Spindle/Laser PWM as TTL or VIN via FET
    - X/Y/Z/Probe inputs via 3-pin headers

    Obused the EXP1/2 Headers for additional
      - coolant 1/2 and spindle enable outputs
      - 3 control buttons 

    Had to set upload_speed = 460800 in platformio.ini to make the flashing work.

    2021 der-Frickler.net

    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.
*/

#define MACHINE_NAME            "MKS-DLC32"

// OLED connected to the i2c header
#define OLED_SDA GPIO_NUM_0
#define OLED_SCL GPIO_NUM_4
#define DISPLAY_CODE_FILENAME   "Custom/oled_basic.cpp"

// I2S (steppers & other output-only pins)
#define USE_I2S_OUT
#define USE_I2S_STEPS
//#define DEFAULT_STEPPER ST_I2S_STATIC
#define I2S_OUT_BCK      GPIO_NUM_16
#define I2S_OUT_WS       GPIO_NUM_17
#define I2S_OUT_DATA     GPIO_NUM_21

#define GRBL_SPI_SS     GPIO_NUM_15
#define GRBL_SPI_MOSI   GPIO_NUM_13
#define GRBL_SPI_MISO   GPIO_NUM_12
#define GRBL_SPI_SCK    GPIO_NUM_14
#define GRBL_SPI_FREQ 4000000
#define SDCARD_DET_PIN          GPIO_NUM_39 // Sensor_VN

#define X_STEP_PIN              I2SO(1)
#define X_DIRECTION_PIN         I2SO(2)
#define Y_STEP_PIN              I2SO(5)
#define Y_DIRECTION_PIN         I2SO(6)
#define Z_STEP_PIN              I2SO(3)
#define Z_DIRECTION_PIN         I2SO(4)
#define DEFAULT_DIRECTION_INVERT_MASK    (bit(X_AXIS) | bit(Y_AXIS) | bit(Z_AXIS))

#define X_LIMIT_PIN             GPIO_NUM_36 // Sensor_VP
#define Y_LIMIT_PIN             GPIO_NUM_35
#define Z_LIMIT_PIN             GPIO_NUM_34

#define STEPPERS_DISABLE_PIN    I2SO(0)

#define SPINDLE_TYPE            SpindleType::PWM
#define SPINDLE_OUTPUT_PIN      GPIO_NUM_22   // labeled SpinPWM

#define PROBE_PIN               GPIO_NUM_2 

#define COOLANT_MIST_PIN        I2SO(7)     // PIN1 on EXP1 labeled Beeper 
#define COOLANT_FLOOD_PIN       GPIO_NUM_5  // PIN3 on ESP1 labeled LCD_EN
#define SPINDLE_ENABLE_PIN      GPIO_NUM_27 // PIN4 on EXP1 labeled LCD_RS

#define CONTROL_RESET_PIN       GPIO_NUM_19  // PIN1 on EXP2 labeled LCD_MISO
#define CONTROL_FEED_HOLD_PIN   GPIO_NUM_18  // PIN2 on EXP2 labeled LCD_SCK
#define CONTROL_CYCLE_START_PIN GPIO_NUM_23  // PIN6 on EXP2 labeled LCD_MOSI
