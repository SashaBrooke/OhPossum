#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h" 
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/sync.h" 

#include "pid.h"
#include "as5600.h"
#include "command.h"
#include "gimbal_configuration.h"

#define FREQ2PERIOD(freq) ((freq) != 0 ? (1.0f / (freq)) : 0.0f)
#define SECS2USECS(secs)  ((secs) * 1000000)

#define PAN_I2C_PORT        i2c0
#define PAN_I2C_SDA_PIN     4
#define PAN_I2C_SCL_PIN     5
#define PAN_ENC_DIR_PIN     3
#define PAN_PWM_PIN         6
#define PAN_MOTOR_DIR_PIN   8

// #define TILT_I2C_PORT       i2c1
// #define TILT_I2C_SDA_PIN    10
// #define TILT_I2C_SCL_PIN    11
// #define TILT_ENC_DIR_PIN    12
// #define TILT_PWM_PIN        7
// #define TILT_MOTOR_DIR_PIN  9

#define TEST_PIN 0

#define PWM_TOP_REG 100
#define PWM_CLK_DIVIDER 125.0f

#define CONTROLS_FREQ 1000

typedef struct repeating_timer repeating_timer_t;

// Variables
volatile uint16_t panPos = 0;
volatile float panPid = 1.0f;
volatile uint8_t panDir = 0;

// volatile uint16_t tiltPos = 0;
// volatile float tiltPid = 1.0f;
// volatile uint8_t tiltDir = 0;

volatile bool printFlag = false;

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

    // Configure debugging pin
    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);  /**< Default LOW, just be explicit */
}

bool updateMotors(repeating_timer_t *timer) {
    // Timing debug
    gpio_put(TEST_PIN, 1);

    gimbal_t *gimbal = (gimbal_t *)timer->user_data;

    // Pan
    uint16_t panRawAngle = AS5600_getRawAngle(&gimbal->panEncoder);
    panPos = panRawAngle;

    if (gimbal->gimbalMode == GIMBAL_MODE_ARMED) {
        float panPidOutput = PID_update(&gimbal->panPositionController, 4000.0f, (float)panRawAngle);
        // float panPwmDutyCycle = PID_normaliseOutput(&gimbal->panPositionController, -100.0f, 100.0f);
        uint8_t panDirection = panPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
        gpio_put(PAN_MOTOR_DIR_PIN, panDirection);
        pwm_set_gpio_level(PAN_PWM_PIN, (uint16_t)(abs(panPidOutput)));

        panPid = panPidOutput;
        panDir = panDirection;
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

    //     tiltPid = tiltPidOutput;
    //     tiltDir = tiltDirection;
    // }

    // Handle print flag toggling
    static int counter = 0; 
    counter++;
    if (counter >= gimbal->streamRate) { // 5
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
    loadGimbalConfiguration(&gimbalConfig);
    displayGimbalConfiguration(&gimbalConfig);

    gimbal.panEncoder = AS5600_setup(PAN_I2C_PORT, PAN_ENC_DIR_PIN, AS5600_CLOCK_WISE);
    // gimbal.tiltEncoder = AS5600_setup(TILT_I2C_PORT, TILT_DIR_PIN, AS5600_CLOCK_WISE);

    // Load PID values or set to default

    gimbal.panPositionController = PID_setup(1.0f, 0.0f, 0.0f, 
                                             0.0f, 
                                             -100.0f, 100.0f, 
                                             0.0f, 0.0f, 
                                             FREQ2PERIOD(CONTROLS_FREQ), 
                                             (float)AS5600_RAW_ANGLE_RESOLUTION);
    gimbalConfig.panPositionController = gimbal.panPositionController;
    // gimbal.tiltPositionController = PID_setup(1.0f, 0.0f, 0.0f, 
    //                                           0.0f, 
    //                                           -100.0f, 100.0f, 
    //                                           0.0f, 0.0f, 
    //                                           FREQ2PERIOD(CONTROLS_FREQ), 
    //                                           (float)AS5600_RAW_ANGLE_RESOLUTION);
    // gimbalConfig.tiltPositionController = gimbal.tiltPositionController;

    displayGimbal(&gimbal);

    printf("Starting controls loop\n");
    repeating_timer_t timer; // Maybe use real time clock peripheral if for use longer than ~72 mins
    add_repeating_timer_us(-SECS2USECS(FREQ2PERIOD(CONTROLS_FREQ)), updateMotors, &gimbal, &timer);

    // Enable serial commands
    initSerialCommandInput();
    char* command;

    while(true) {
        if (gimbal.streaming && printFlag) {
            printf("%u,%f,%u\n", panPos, panPid, panDir);
            printFlag = false;  /**< Reset print flag */
        }

        command = readSerialCommand_nonBlocking();
        if (command != NULL) {
            // Handle command (agrument parser)
            //

            // Change for proper command handling
            if (gimbal.gimbalMode == GIMBAL_MODE_FREE) {
                gimbal.gimbalMode = GIMBAL_MODE_ARMED;
                gimbal.savedConfiguration = false;
            } else if (gimbal.gimbalMode == GIMBAL_MODE_ARMED) {
                gimbal.gimbalMode = GIMBAL_MODE_FREE;
                gimbal.savedConfiguration = false;
            }
            displayGimbal(&gimbal);

            // cancel_repeating_timer(&timer);

            // saveGimbalConfiguration(&gimbalConfig);
            // gimbal_configuration_t tmpConfig;
            // loadGimbalConfiguration(&tmpConfig);
            // printf("Temp Serial Number (after save): %d\n", tmpConfig.serialNumber);

            // add_repeating_timer_us(-SECS2USECS(FREQ2PERIOD(CONTROLS_FREQ)), updateMotors, NULL, &timer);
        }
    }

    return 0;
}
