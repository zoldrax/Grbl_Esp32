#pragma once
// clang-format off

/*
    makeit.lu Board for grbl ESP32
    https://der-frickler.net/technik/cnc_controller

    Fits below the very cheap GRBL Arduino UNO Shield and replaces the Arduino UNO by an ESP32

    Up to 4 independent Stepper Axis
    4/2 Outputs - Spindle PWM, Coolant 1/2
    4 Inputs + 4 Buttons

    2020 der-Frickler.net/makeit.lu

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

#define MACHINE_NAME            "Milly4X"

#define N_AXIS 4

#define X_STEP_PIN              GPIO_NUM_12
#define X_DIRECTION_PIN         GPIO_NUM_14
#define Y_STEP_PIN              GPIO_NUM_26
#define Y_DIRECTION_PIN         GPIO_NUM_15
#define Z_STEP_PIN              GPIO_NUM_27
#define Z_DIRECTION_PIN         GPIO_NUM_33

#define X_LIMIT_PIN             GPIO_NUM_17
#define Y_LIMIT_PIN             GPIO_NUM_4
#define Z_LIMIT_PIN             GPIO_NUM_16

#define A_STEP_PIN              GPIO_NUM_22
#define A_DIRECTION_PIN         GPIO_NUM_2
//#define A_LIMIT_PIN             GPIO_NUM_32

// OK to comment out to use pin for other features
#define STEPPERS_DISABLE_PIN GPIO_NUM_13

#define SPINDLE_TYPE            SpindleType::PWM
#define SPINDLE_OUTPUT_PIN    	  GPIO_NUM_21

//#define SPINDLE_OUTPUT_PIN      GPIO_NUM_2   // labeled SpinPWM
//#define SPINDLE_ENABLE_PIN      GPIO_NUM_22  // labeled SpinEnbl

//#define COOLANT_MIST_PIN        GPIO_NUM_21  // labeled Mist
#define COOLANT_FLOOD_PIN 	GPIO_NUM_25  // labeled Flood
#define ATC_RELEASE_PIN       GPIO_NUM_25  // labeled Flood

#define PROBE_PIN       	GPIO_NUM_32

#define CONTROL_SAFETY_DOOR_PIN   GPIO_NUM_35  // needs external pullup
#define CONTROL_RESET_PIN         GPIO_NUM_34  // needs external pullup
#define CONTROL_FEED_HOLD_PIN     GPIO_NUM_39  // needs external pullup
//#define CONTROL_CYCLE_START_PIN   GPIO_NUM_36  // needs external pullup

#define DEFAULT_HOMING_CYCLE_0 bit(Z_AXIS)
#define DEFAULT_HOMING_CYCLE_1 bit(X_AXIS)
#define DEFAULT_HOMING_CYCLE_2 bit(Y_AXIS)

#define MACRO_BUTTON_0_PIN		GPIO_NUM_36
//#define MACRO_BUTTON_1_PIN		GPIO_NUM_39
//#define MACRO_BUTTON_2_PIN		GPIO_NUM_34
//#define MACRO_BUTTON_2_PIN		GPIO_NUM_35
#ifdef INVERT_CONTROL_PIN_MASK
    #undef INVERT_CONTROL_PIN_MASK
#endif
// Macro3 | Macro2 | Macro 1| Macro 0 |Cycle Start | Feed Hold | Reset | Safety Door
#define INVERT_CONTROL_PIN_MASK B11111111

// ============= End CPU MAP ==================
