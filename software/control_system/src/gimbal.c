/**
 * @file gimbal.c
 * @brief Source file for gimbal controls system module.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h" 
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/sync.h" 
#include "hardware/timer.h"

#include "pid.h"
#include "as5600.h"
#include "command.h"
#include "gimbal_configuration.h"

#define FREQ2PERIOD(freq) ((freq) != 0 ? (1.0f / (freq)) : 0.0f) // Converts a frequency in Hz to a period in seconds
#define SECS2USECS(secs)  ((secs) * 1000000)                     // Converts a time from seconds to micro-seconds

// Pan hardware
#define PAN_I2C_PORT         i2c0
#define PAN_I2C_SDA_PIN      4
#define PAN_I2C_SCL_PIN      5
#define PAN_ENC_DIR_PIN      3
#define PAN_PWM_PIN          6
#define PAN_MOTOR_DIR_PIN    8
#define PAN_LOWER_LIMIT_PIN  17
#define PAN_UPPER_LIMIT_PIN  16

// // Tilt hardware
// #define TILT_I2C_PORT         i2c1
// #define TILT_I2C_SDA_PIN      10
// #define TILT_I2C_SCL_PIN      11
// #define TILT_ENC_DIR_PIN      12
// #define TILT_PWM_PIN          7
// #define TILT_MOTOR_DIR_PIN    9
// #define TILT_LOWER_LIMIT_PIN  14
// #define TILT_UPPER_LIMIT_PIN  15

// Debug hardware
#define TEST_PIN 22

// PWM settings
#define PWM_TOP_REG 100
#define PWM_CLK_DIVIDER 125.0f

// Controls loop frequency
#define CONTROLS_FREQ 1000

// Homing
#define HOMING_SPEED 0.1    // (angular position / second)
#define HOMING_DISTANCE 40  // (angular position)
#define HOMING_TIMEOUT 10   // (seconds)

// Stream variables
volatile uint16_t panPos = 0;
volatile float panMotor = 0.0f;
volatile uint8_t panDir = 0;

// volatile uint16_t tiltPos = 0;
// volatile float tiltMotor = 0.0f;
// volatile uint8_t tiltDir = 0;

volatile bool printFlag = false;

/**
 * @brief Configures GPIO pins and peripherals for I2C, PWM, and debugging.
 */
void setupGPIO() {
    // Configure I2C Communication
    i2c_init(PAN_I2C_PORT, 400000);
    gpio_set_function(PAN_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PAN_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PAN_I2C_SDA_PIN);
    gpio_pull_up(PAN_I2C_SCL_PIN);

    // i2c_init(TILT_I2C_PORT, 400000);
    // gpio_set_function(TILT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    // gpio_set_function(TILT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // gpio_pull_up(TILT_I2C_SDA_PIN);
    // gpio_pull_up(TILT_I2C_SCL_PIN);

    // Configure PWM output
    gpio_set_function(PAN_PWM_PIN, GPIO_FUNC_PWM);
    // gpio_set_function(TILT_PWM_PIN, GPIO_FUNC_PWM);

    unsigned int pwmSliceNum = pwm_gpio_to_slice_num(PAN_PWM_PIN); // Pan and tilt PWM pins are on the same slice
    pwm_config configPWM = pwm_get_default_config();
    pwm_config_set_clkdiv(&configPWM, PWM_CLK_DIVIDER);
    pwm_init(pwmSliceNum, &configPWM, true);
    pwm_set_wrap(pwmSliceNum, PWM_TOP_REG - 1); 

    pwm_set_gpio_level(PAN_PWM_PIN, 0);    /**< Explicitly set to 0 duty cycle initially */
    // pwm_set_gpio_level(TILT_PWM_PIN, 0);   /**< Explicitly set to 0 duty cycle initially */
    pwm_set_enabled(pwmSliceNum, true);

    // Configure initial motor directions
    gpio_init(PAN_MOTOR_DIR_PIN);
    gpio_set_dir(PAN_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_put(PAN_MOTOR_DIR_PIN, 0);  /**< Default LOW, just be explicit */

    // gpio_init(TILT_MOTOR_DIR_PIN);
    // gpio_set_dir(TILT_MOTOR_DIR_PIN, GPIO_OUT);
    // gpio_put(TILT_MOTOR_DIR_PIN, 0);  /**< Default LOW, just be explicit */

    // Configure limit switch pins
    gpio_init(PAN_LOWER_LIMIT_PIN);
    gpio_set_dir(PAN_LOWER_LIMIT_PIN, GPIO_IN);
    gpio_pull_up(PAN_LOWER_LIMIT_PIN);

    gpio_init(PAN_UPPER_LIMIT_PIN);
    gpio_set_dir(PAN_UPPER_LIMIT_PIN, GPIO_IN);
    gpio_pull_up(PAN_UPPER_LIMIT_PIN);

    // gpio_init(TILT_LOWER_LIMIT_PIN);
    // gpio_set_dir(TILT_LOWER_LIMIT_PIN, GPIO_IN);
    // gpio_pull_up(TILT_LOWER_LIMIT_PIN);

    // gpio_init(TILT_UPPER_LIMIT_PIN);
    // gpio_set_dir(TILT_UPPER_LIMIT_PIN, GPIO_IN);
    // gpio_pull_up(TILT_UPPER_LIMIT_PIN);

    // Configure debugging pin
    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);  /**< Default LOW, just be explicit */
}

/**
 * @brief Set soft limits for each gimbal axis.
 */
void setupAxisLimits(gimbal_t *gimbal) {
    uint64_t startTime = time_us_64();

    // Temp limits (gimbal limits only set if homing process actually completes)
    float tmpPanLowerLimit;
    float tmpPanUpperLimit;
    float tmpTiltLowerLimit;
    float tmpTiltUpperLimit;

    // Arm gimbal
    gimbal->panPositionSetpoint = (float)panPos;
    // uint16_t tiltStartPos = AS5600_getRawAngle(&gimbal->tiltEncoder);
    // gimbal->tiltPositionSetpoint = (float)tiltStartPos;
    gimbal->gimbalMode = GIMBAL_MODE_ARMED;

    // Find pan lower limit
    while (true) {
        bool atPanLowerLimit = !gpio_get(PAN_LOWER_LIMIT_PIN); // Pulled high default (therefore !)

        if (time_us_64() - startTime >= SECS2USECS(HOMING_TIMEOUT)) {
            printf("Timeout: Homing process exceeded time limit. Exiting homing process.\n");
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            return;
        }

        if (atPanLowerLimit) {
            tmpPanLowerLimit = (float)panPos; // + some safe offset
            printf("#### Lower limit found at %.1f ####\n", tmpPanLowerLimit);
            // gimbal->panEncoder.offset = gimbal->panLowerLimit;
            break;
        } else {
            float newPanSetpoint = gimbal->panPositionSetpoint - HOMING_SPEED;
            gimbal->panPositionSetpoint = newPanSetpoint >= 0 ? newPanSetpoint : newPanSetpoint + AS5600_RAW_ANGLE_MAX;
            printf("Moving to lower pan limit (actual: %u, setpoint: %.1f)\n", panPos, gimbal->panPositionSetpoint); // Hacky
        }
    }

    sleep_ms(1000);

    // Find pan upper limit
    int iter = 0;
    while (true) {
        bool atPanUpperLimit = !gpio_get(PAN_UPPER_LIMIT_PIN); // Pulled high default (therefore !)

        if (time_us_64() - startTime >= SECS2USECS(HOMING_TIMEOUT)) {
            printf("Timeout: Homing process exceeded time limit. Exiting homing process.\n");
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            return;
        }

        if (atPanUpperLimit) {
            tmpPanUpperLimit = (float)panPos; // + some safe offset
            printf("#### Upper limit found at %.1f ####\n", tmpPanUpperLimit);
            break;
        } else {
            float newPanSetpoint = gimbal->panPositionSetpoint + HOMING_SPEED;
            gimbal->panPositionSetpoint = newPanSetpoint <= AS5600_RAW_ANGLE_MAX ? newPanSetpoint : newPanSetpoint - AS5600_RAW_ANGLE_MAX;
            printf("Moving to upper pan limit (actual: %u, setpoint: %.1f)\n", panPos, gimbal->panPositionSetpoint); // Hacky
        }
    }

    sleep_ms(1000);

    // Move to a central position
    float panMidpoint;
    if (tmpPanLowerLimit < tmpPanUpperLimit) {
        panMidpoint = (tmpPanLowerLimit + tmpPanUpperLimit) / 2;
    } else {
        panMidpoint = fmod(tmpPanLowerLimit + (AS5600_RAW_ANGLE_MAX + tmpPanUpperLimit - tmpPanLowerLimit) / 2, AS5600_RAW_ANGLE_MAX);
    }
    gimbal->panPositionSetpoint = panMidpoint >= 0 ? panMidpoint : panMidpoint + AS5600_RAW_ANGLE_MAX;
    printf("Midpoint: %.0f\n", gimbal->panPositionSetpoint);

    while (true) {
        if (time_us_64() - startTime >= SECS2USECS(HOMING_TIMEOUT)) {
            printf("Timeout: Homing process exceeded time limit. Exiting homing process.\n");
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            return;
        }

        if (fabs((float)panPos - panMidpoint) < HOMING_DISTANCE) {
            printf("At midpoint (%u actual vs %.1f setpoint)\n", panPos, panMidpoint);

            // Actually set soft limits
            gimbal->panLowerLimit = tmpPanLowerLimit;
            gimbal->panUpperLimit = tmpPanUpperLimit;
            // gimbal->tiltLowerLimit = tmpTiltLowerLimit;
            // gimbal->tiltUpperLimit = tmpTiltUpperLimit;

            sleep_ms(500); // Pause at midpoint

            // Disarm
            gimbal->gimbalMode = GIMBAL_MODE_FREE;
            break;
        } else {
            printf("Moving to midpoint (actual: %u, setpoint: %.1f)\n", panPos, gimbal->panPositionSetpoint); // Hacky
        }
    }

    // Tilt implementation

    printf("Finished setting axis soft limits.\n");
}

/**
 * @brief Timer callback to perform the gimbal control system loop.
 * 
 * @param timer Pointer to the repeating timer instance calling the callback.
 * @return Always returns true to keep the timer active.
 */
bool updateMotors(repeating_timer_t *timer) {
    // Timing debug
    gpio_put(TEST_PIN, 1);

    // Get gimbal state
    gimbal_t *gimbal = (gimbal_t *)timer->user_data;

    // Pan
    uint16_t panRawAngle = AS5600_getRawAngle(&gimbal->panEncoder);
    panPos = panRawAngle;

    if (gimbal->gimbalMode == GIMBAL_MODE_ARMED) {
        float panPidOutput = PID_update(&gimbal->panPositionController, gimbal->panPositionSetpoint, (float)panRawAngle);
        float panPwmDutyCycle = PID_normaliseOutput(&gimbal->panPositionController, -100.0f, 100.0f);
        uint8_t panDirection = panPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
        gpio_put(PAN_MOTOR_DIR_PIN, panDirection);
        pwm_set_gpio_level(PAN_PWM_PIN, (uint16_t)(abs(panPidOutput)));

        panMotor = panPwmDutyCycle;
        panDir = panDirection;
    } else if (gimbal->gimbalMode = GIMBAL_MODE_FREE) {
        PID_reset(&gimbal->panPositionController);
        gpio_put(PAN_MOTOR_DIR_PIN, 0); // default dir
        pwm_set_gpio_level(PAN_PWM_PIN, 0); // default pwm

        panMotor = 0;
        panDir = 0;
    }

    // Tilt
    // uint16_t tiltRawAngle = AS5600_getRawAngle(&gimbal->tiltEncoder);
    // tiltPos = tiltRawAngle;

    // if (gimbalMode == GIMBAL_MODE_ARMED) {
    //     float tiltPidOutput = PID_update(&gimbal->tiltPositionController, 1000.0f, (float)tiltRawAngle);
    //     float tiltPwmDutyCycle = PID_normaliseOutput(&gimbal->tiltPositionController, -100.0f, 100.0f);
    //     uint8_t tiltDirection = tiltPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
    //     gpio_put(TILT_MOTOR_DIR_PIN, tiltDirection);
    //     pwm_set_gpio_level(TILT_PWM_PIN, PWM_TOP_REG * abs(panPwmDutyCycle));

    //     tiltMotor = tiltPwmDutyCycle;
    //     tiltDir = tiltDirection;
    // }

    // Handle print flag toggling
    static int counter = 0; 
    counter++;
    if (counter >= gimbal->streamRate) {
        printFlag = true;
        counter = 0;
    }

    // Timing debug
    gpio_put(TEST_PIN, 0);

    return true;
}

int main() {
    stdio_init_all();

    setupGPIO();

    sleep_ms(3000); // Allow serial monitor to auto-detect COM port
                    // (recommended Arduino IDE for serial monitor/plotter)

    printf("\n ------------------ STARTING GIMBAL ------------------ \n");

    gimbal_t gimbal;
    setupGimbal(&gimbal);

    gimbal_configuration_t gimbalConfig;
    bool loadedPrevConfig = loadGimbalConfiguration(&gimbalConfig);

    if (loadedPrevConfig) {
        // Setup controllers using saved values
        gimbal.panPositionController = PID_setup(gimbalConfig.panPositionController.Kp,
                                                 gimbalConfig.panPositionController.Ki,
                                                 gimbalConfig.panPositionController.Kd,
                                                 gimbalConfig.panPositionController.tau,
                                                 gimbalConfig.panPositionController.outLimMin,
                                                 gimbalConfig.panPositionController.outLimMax,
                                                 gimbalConfig.panPositionController.intLimMin,
                                                 gimbalConfig.panPositionController.intLimMax,
                                                 FREQ2PERIOD(CONTROLS_FREQ),
                                                 (float)AS5600_RAW_ANGLE_MAX);
        // gimbal.tiltPositionController = PID_setup(gimbalConfig.tiltPositionController.Kp,
        //                                           gimbalConfig.tiltPositionController.Ki,
        //                                           gimbalConfig.tiltPositionController.Kd,
        //                                           gimbalConfig.tiltPositionController.tau,
        //                                           gimbalConfig.tiltPositionController.outLimMin,
        //                                           gimbalConfig.tiltPositionController.outLimMax,
        //                                           gimbalConfig.tiltPositionController.intLimMin,
        //                                           gimbalConfig.tiltPositionController.intLimMax,
        //                                           FREQ2PERIOD(CONTROLS_FREQ),
        //                                           (float)AS5600_RAW_ANGLE_MAX);
    } else {
        // Setup using default values
        printf("Creating new configuration using default values.\n");
        gimbalConfig.serialNumber = 0;
        gimbal.panPositionController = PID_setup(0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 0.0f,
                                                 -100.0f,
                                                 100.0f,
                                                 0.0f,
                                                 0.0f,
                                                 FREQ2PERIOD(CONTROLS_FREQ),
                                                 (float)AS5600_RAW_ANGLE_MAX
                                                 );
        // gimbal.tiltPositionController = PID_setup(0.0f,
        //                                           0.0f,
        //                                           0.0f,
        //                                           0.0f,
        //                                           -100.0f,
        //                                           100.0f,
        //                                           0.0f,
        //                                           0.0f,
        //                                           FREQ2PERIOD(CONTROLS_FREQ),
        //                                           (float)AS5600_RAW_ANGLE_MAX);
    }

    gimbalConfig.panPositionController = gimbal.panPositionController;
    // gimbalConfig.tiltPositionController = gimbal.tiltPositionController;

    displayGimbalConfiguration(&gimbalConfig);

    gimbal.panEncoder = AS5600_setup(PAN_I2C_PORT, PAN_ENC_DIR_PIN, AS5600_CLOCK_WISE);
    // gimbal.tiltEncoder = AS5600_setup(TILT_I2C_PORT, TILT_DIR_PIN, AS5600_CLOCK_WISE);

    displayGimbal(&gimbal);

    printf("Starting controls loop\n");
    repeating_timer_t timer; // Consider using real time clock peripheral if for use longer than ~72 mins
    add_repeating_timer_us(-SECS2USECS(FREQ2PERIOD(CONTROLS_FREQ)), updateMotors, &gimbal, &timer);

    sleep_ms(1000);
    setupAxisLimits(&gimbal);
    sleep_ms(1000);

    // Enable serial commands
    resetSerialCommandInput();
    char* command;

    while(true) {
        if (gimbal.streaming && printFlag) {
            // Handle specific stream outputs
            if (gimbal.panPositionStream) {
                printf("%u,", panPos);
                
                if (gimbal.gimbalMode == GIMBAL_MODE_ARMED) {
                    printf("%f,", gimbal.panPositionSetpoint);
                }
            }
            if (gimbal.panPidStream) {
                printf("%f,%f,%f,%f,",
                    gimbal.panPositionController.prevError, // Actually the error from the current controls loop
                    gimbal.panPositionController.integrator,
                    gimbal.panPositionController.differentiator,
                    gimbal.panPositionController.output
                );
            }
            if (gimbal.panMotorStream) {
                printf("%f,%u,", panMotor, panDir);
            }
            printf("\n");

            printFlag = false;  /**< Reset print flag */
        }

        command = readSerialCommand_nonBlocking();
        if (command != NULL) {
            cancel_repeating_timer(&timer);
            processCommands(command, &gimbal, &gimbalConfig);
            add_repeating_timer_us(-SECS2USECS(FREQ2PERIOD(CONTROLS_FREQ)), updateMotors, &gimbal, &timer);
            resetSerialCommandInput();
        }
    }

    return 0;
}
