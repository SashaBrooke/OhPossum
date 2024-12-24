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

#define CONTROLS_FREQ 1000

typedef struct repeating_timer repeating_timer_t;

// Control mode
gimbal_mode_t gimbalMode;

// Encoders
volatile AS5600_t panEncoder;
// volatile AS5600_t tiltEncoder;

// PID Controllers
volatile PID_t panPositionController;
// volatile PID_t tiltPositionController;

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
    pwm_config_set_clkdiv(&configPWM, 125.f);
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

    // Pan
    uint16_t panRawAngle = AS5600_getRawAngle(&panEncoder);
    panPos = panRawAngle;

    if (gimbalMode == GIMBAL_MODE_ARMED) {
        float panPidOutput = PID_update(&panPositionController, 4000.0f, (float)panRawAngle);
        // float panPwmDutyCycle = PID_normaliseOutput(&panPositionController, -100.0f, 100.0f);
        uint8_t panDirection = panPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
        gpio_put(PAN_MOTOR_DIR_PIN, panDirection);
        pwm_set_gpio_level(PAN_PWM_PIN, (uint16_t)(abs(panPidOutput)));

        panPid = panPidOutput;
        panDir = panDirection;
    }

    // Tilt
    // uint16_t tiltRawAngle = AS5600_getRawAngle(&tiltEncoder);
    // tiltPos = tiltRawAngle;

    // if (gimbalMode == GIMBAL_MODE_ARMED) {
    //     float tiltPidOutput = PID_update(&tiltPositionController, 1000.0f, (float)tiltRawAngle);
    //     float tiltPwmDutyCycle = PID_normaliseOutput(&tiltPositionController, -100.0f, 100.0f);
    //     uint8_t tiltDirection = tiltPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
    //     gpio_put(TILT_MOTOR_DIR_PIN, tiltDirection);
    //     pwm_set_gpio_level(TILT_PWM_PIN, PWM_TOP_REG * abs(panPwmDutyCycle));

    //     tiltPid = tiltPidOutput;
    //     tiltDir = tiltDirection;
    // }

    // Handle print flag toggling
    static int counter = 0; 
    counter++;
    if (counter >= 1000) { // 5
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

    sleep_ms(5000); // Allow time to open serial monitor

    gimbal_configuration_t gimbalConfig;

    loadGimbalConfiguration(&gimbalConfig);
    
    // Print gimbal serial number
    printf("Gimbal Serial Number: %d\n", gimbalConfig.serialNumber);

    // Print pan position controller PID values
    printf("Pan Position Controller:\n");
    printf("  Kp: %.2f\n", gimbalConfig.panPositionController.Kp);
    printf("  Ki: %.2f\n", gimbalConfig.panPositionController.Ki);
    printf("  Kd: %.2f\n", gimbalConfig.panPositionController.Kd);
    printf("  tau: %.2f\n", gimbalConfig.panPositionController.tau);
    printf("  outLimMin: %.2f\n", gimbalConfig.panPositionController.outLimMin);
    printf("  outLimMax: %.2f\n", gimbalConfig.panPositionController.outLimMax);
    printf("  intLimMin: %.2f\n", gimbalConfig.panPositionController.intLimMin);
    printf("  intLimMax: %.2f\n", gimbalConfig.panPositionController.intLimMax);
    printf("  T: %.2f\n", gimbalConfig.panPositionController.T);
    printf("  integrator: %.2f\n", gimbalConfig.panPositionController.integrator);
    printf("  prevError: %.2f\n", gimbalConfig.panPositionController.prevError);
    printf("  differentiator: %.2f\n", gimbalConfig.panPositionController.differentiator);
    printf("  prevMeasurement: %.2f\n", gimbalConfig.panPositionController.prevMeasurement);
    printf("  output: %.2f\n", gimbalConfig.panPositionController.output);
    printf("  maxMeasurement: %.2f\n", gimbalConfig.panPositionController.maxMeasurement);

    // // Print tilt position controller PID values
    // printf("Tilt Position Controller:\n");
    // printf("  Kp: %.2f\n", gimbalConfig.tiltPositionController.Kp);
    // printf("  Ki: %.2f\n", gimbalConfig.tiltPositionController.Ki);
    // printf("  Kd: %.2f\n", gimbalConfig.tiltPositionController.Kd);
    // printf("  tau: %.2f\n", gimbalConfig.tiltPositionController.tau);
    // printf("  outLimMin: %.2f\n", gimbalConfig.tiltPositionController.outLimMin);
    // printf("  outLimMax: %.2f\n", gimbalConfig.tiltPositionController.outLimMax);
    // printf("  intLimMin: %.2f\n", gimbalConfig.tiltPositionController.intLimMin);
    // printf("  intLimMax: %.2f\n", gimbalConfig.tiltPositionController.intLimMax);
    // printf("  T: %.2f\n", gimbalConfig.tiltPositionController.T);
    // printf("  integrator: %.2f\n", gimbalConfig.tiltPositionController.integrator);
    // printf("  prevError: %.2f\n", gimbalConfig.tiltPositionController.prevError);
    // printf("  differentiator: %.2f\n", gimbalConfig.tiltPositionController.differentiator);
    // printf("  prevMeasurement: %.2f\n", gimbalConfig.tiltPositionController.prevMeasurement);
    // printf("  output: %.2f\n", gimbalConfig.tiltPositionController.output);
    // printf("  maxMeasurement: %.2f\n", gimbalConfig.tiltPositionController.maxMeasurement);

    gimbalMode = GIMBAL_MODE_FREE; // Safety: Force gimbal into free mode on powerup
    gimbalConfig.serialNumber = 1;

    // May want a better method of doing this for additional modes
    if (gimbalMode == GIMBAL_MODE_FREE) {
        printf("Gimbal Mode: GIMBAL_MODE_FREE\n");
    } else if (gimbalMode == GIMBAL_MODE_ARMED) {
        printf("Gimbal Mode: GIMBAL_MODE_ARMED\n");
    } else {
        printf("Gimbal Mode: UNKNOWN\n");
    }

    panEncoder = AS5600_setup(PAN_I2C_PORT, PAN_ENC_DIR_PIN, AS5600_CLOCK_WISE);
    // gimbalConfig.tiltEncoder = AS5600_setup(TILT_I2C_PORT, TILT_DIR_PIN, AS5600_CLOCK_WISE);

    // Load PID values or set to default

    panPositionController = PID_setup(1.0f, 0.0f, 0.0f, 
                                      0.0f, 
                                      -100.0f, 100.0f, 
                                      0.0f, 0.0f, 
                                      (1.0f / CONTROLS_FREQ), 
                                      (float)AS5600_RAW_ANGLE_RESOLUTION);
    gimbalConfig.panPositionController = panPositionController;
    // tiltPositionController = PID_setup(1.0f, 0.0f, 0.0f, 
    //                                    0.0f, 
    //                                    -100.0f, 100.0f, 
    //                                    0.0f, 0.0f, 
    //                                    (1.0f / CONTROLS_FREQ), 
    //                                    (float)AS5600_RAW_ANGLE_RESOLUTION);
    // gimbalConfig.tiltPositionController = tiltPositionController;

    // // Magnet debugging
    // while (!AS5600_magnetGood(&panEncoder)) {
    //     printf("Magnet not right...\n");
    //     sleep_ms(1000);
    // }
    // printf("Magnet good!\n");

    printf("Starting timer\n");
    repeating_timer_t timer; // Maybe use real time clock peripheral if for use longer than ~72 mins
    add_repeating_timer_us(-CONTROLS_FREQ, updateMotors, NULL, &timer);

    // Enable serial commands
    initSerialCommandInput();
    char* command;

    while(true) {
        if (printFlag) {
            printf("%u,%f,%u\n", panPos, panPid, panDir);
            printFlag = false;  /**< Reset print flag */
        }

        command = readSerialCommand_nonBlocking();
        if (command != NULL) {
            // Handle command (agrument parser)
            //

            cancel_repeating_timer(&timer);

            saveGimbalConfiguration(&gimbalConfig);
            gimbal_configuration_t tmpConfig;
            loadGimbalConfiguration(&tmpConfig);
            printf("Temp Serial Number (after save): %d\n", tmpConfig.serialNumber);

            add_repeating_timer_us(-CONTROLS_FREQ, updateMotors, NULL, &timer);
        }
    }

    return 0;
}
