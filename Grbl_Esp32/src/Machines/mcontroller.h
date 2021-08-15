#pragma once
// clang-format off

#define MACHINE_NAME "CPU_MAP_MCONTROLLER"
#define SHOW_EXTENDED_SETTINGS

//#define USE_GANGED_AXES // allow two motors on an axis

#define X_STEP_PIN    GPIO_NUM_17
#define Y_STEP_PIN    GPIO_NUM_4
#define Z_STEP_PIN      GPIO_NUM_27
#define A_STEP_PIN      GPIO_NUM_12

#define X_DIRECTION_PIN   GPIO_NUM_26
#define Y_DIRECTION_PIN   GPIO_NUM_25
#define Z_DIRECTION_PIN   GPIO_NUM_33
#define A_DIRECTION_PIN   GPIO_NUM_14

#define STEPPERS_DISABLE_PIN GPIO_NUM_13

//#define SPINDLE_PWM_PIN    GPIO_NUM_2
//#define SPINDLE_ENABLE_PIN	GPIO_NUM_32

//#define SPINDLE_PWM_CHANNEL 0	
//#define SPINDLE_PWM_BIT_PRECISION 8   // be sure to match this with SPINDLE_PWM_MAX_VALUE


// Note: Only uncomment this if USE_SPINDLE_RELAY is commented out.
// Relay can be used for spindle or either coolant
//#define COOLANT_FLOOD_PIN 	GPIO_NUM_2
//#define COOLANT_MIST_PIN   	GPIO_NUM_2

#undef DISABLE_LIMIT_PIN_PULL_UP
#define X_LIMIT_PIN      	GPIO_NUM_16
#define Y_LIMIT_PIN      	GPIO_NUM_3
#define Z_LIMIT_PIN     	GPIO_NUM_15
#define A_LIMIT_PIN     	GPIO_NUM_36
#define LIMIT_MASK      	B1111

#define ENABLE_SOFTWARE_DEBOUNCE //TODO: should not be needed, but it is so...

#define PROBE_PIN       	GPIO_NUM_35

// The default value in config.h is wrong for this controller
#ifdef INVERT_CONTROL_PIN_MASK
    #undef INVERT_CONTROL_PIN_MASK			
#endif

#define INVERT_CONTROL_PIN_MASK   B1110

// Note: default is #define IGNORE_CONTROL_PINS in config.h
// uncomment to these lines to use them		
#ifdef IGNORE_CONTROL_PINS
    #undef IGNORE_CONTROL_PINS
#endif

#define CONTROL_RESET_PIN         GPIO_NUM_34  // needs external pullup
//#define CONTROL_FEED_HOLD_PIN     GPIO_NUM_36  // needs external pullup
//#define CONTROL_CYCLE_START_PIN   GPIO_NUM_39  // needs external pullup    

#define DEFAULT_STEP_PULSE_MICROSECONDS 3 
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 //  255 = Keep steppers on 

#define DEFAULT_STEPPING_INVERT_MASK 0 // uint8_t
#define DEFAULT_DIRECTION_INVERT_MASK 0 // uint8_t
#define DEFAULT_INVERT_ST_ENABLE 0 // boolean
#define DEFAULT_INVERT_LIMIT_PINS 0 // boolean
#define DEFAULT_INVERT_PROBE_PIN 0 // boolean 

#define DEFAULT_STATUS_REPORT_MASK 1

#define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
#define DEFAULT_ARC_TOLERANCE 0.002 // mm
#define DEFAULT_REPORT_INCHES 0 // false

#define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
#define DEFAULT_HARD_LIMIT_ENABLE 0  // false

#define DEFAULT_HOMING_ENABLE 1  // false
#define DEFAULT_HOMING_DIR_MASK 3 // move positive dir Z,negative X,Y
#define DEFAULT_HOMING_FEED_RATE 600.0 // mm/min
#define DEFAULT_HOMING_SEEK_RATE 2000.0 // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF 1.5 // mm

#define DEFAULT_SPINDLE_RPM_MAX 1.0 // must be 1 so PWM duty is alway 100% to prevent relay damage

#define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm

#define DEFAULT_LASER_MODE 0 // false

#define DEFAULT_X_STEPS_PER_MM 200.0
#define DEFAULT_Y_STEPS_PER_MM 200.0
#define DEFAULT_Z_STEPS_PER_MM 800.0
#define DEFAULT_A_STEPS_PER_MM 50.0

#define DEFAULT_X_MAX_RATE 8000.0 // mm/min
#define DEFAULT_Y_MAX_RATE 8000.0 // mm/min
#define DEFAULT_Z_MAX_RATE 3000.0 // mm/min
#define DEFAULT_A_MAX_RATE 250.0 // mm/min

#define DEFAULT_X_ACCELERATION (200.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Y_ACCELERATION (200.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Z_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_A_ACCELERATION 100.0 // mm/sec^2

#define DEFAULT_X_MAX_TRAVEL 500.0 // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL 500.0 // mm NOTE: Must be a positive value.
#define DEFAULT_Z_MAX_TRAVEL 80.0 // mm NOTE: Must be a positive value.
#define DEFAULT_A_MAX_TRAVEL 10.0 // mm NOTE: Must be a positive value.
