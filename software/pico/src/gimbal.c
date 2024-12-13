#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h" 
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/sync.h" 

#include "pid.h"
#include "as5600.h"
#include "command_input.h"

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

// Serial input
#define LF          10
#define CR          13
#define NO_VAL      254
#define ENDSTDIN    255

typedef struct repeating_timer repeating_timer_t;

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

bool updateMotors(repeating_timer_t *timer) {
    // Timing debug
    gpio_put(TEST_PIN, 1);

    // Pan
    uint16_t panRawAngle = AS5600_getRawAngle(&panEncoder);
    float panPidOutput = PID_update(&panPositionController, 4000.0f, (float)panRawAngle);
    // float panPwmDutyCycle = PID_normaliseOutput(&panPositionController, -100.0f, 100.0f);
    uint8_t panDirection = panPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
    gpio_put(PAN_MOTOR_DIR_PIN, panDirection);
    pwm_set_gpio_level(PAN_PWM_PIN, (uint16_t)(abs(panPidOutput)));

    // Tilt
    // uint16_t tiltRawAngle = AS5600_getRawAngle(&tiltEncoder);
    // float tiltPidOutput = PID_update(&tiltPositionController, 1000.0f, (float)tiltRawAngle);
    // float tiltPwmDutyCycle = PID_normaliseOutput(&tiltPositionController, -100.0f, 100.0f);
    // uint8_t tiltDirection = tiltPidOutput > 0 ? AS5600_CLOCK_WISE : AS5600_COUNTERCLOCK_WISE;
    // gpio_put(TILT_MOTOR_DIR_PIN, tiltDirection);
    // pwm_set_gpio_level(TILT_PWM_PIN, PWM_TOP_REG * abs(panPwmDutyCycle));

    // Set volatile global variables and handle printFlag
    panPos = panRawAngle;
    panPid = panPidOutput;
    panDir = panDirection;

    // tiltPos = tiltRawAngle;
    // tiltPid = tiltPidOutput;
    // tiltDir = tiltDirection;

    // Handle print flag toggling
    static int counter = 0; 
    counter++;
    if (counter >= 5) {
        printFlag = true;
        counter = 0;
    }

    // Timing debug
    gpio_put(TEST_PIN, 0);

    return true;
}

int main() {
    stdio_init_all();

    sleep_ms(10000); // Allow time to open serial monitor

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

    panEncoder = AS5600_setup(PAN_I2C_PORT, PAN_ENC_DIR_PIN, AS5600_CLOCK_WISE);
    // tiltEncoder = AS5600_setup(TILT_I2C_PORT, TILT_DIR_PIN, AS5600_CLOCK_WISE);

    panPositionController = PID_setup(1.0f, 0.0f, 0.0f, 
                                      0.0f, 
                                      -100.0f, 100.0f, 
                                      0.0f, 0.0f, 
                                      (1.0f / CONTROLS_FREQ), 
                                      (float)AS5600_RAW_ANGLE_RESOLUTION);
    // tiltPositionController = PID_setup(1.0f, 0.0f, 0.0f, 
    //                                   0.0f, 
    //                                   -100.0f, 100.0f, 
    //                                   0.0f, 0.0f, 
    //                                   (1.0f / CONTROLS_FREQ), 
    //                                   (float)AS5600_RAW_ANGLE_RESOLUTION);

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

    gpio_init(PAN_MOTOR_DIR_PIN);
    gpio_set_dir(PAN_MOTOR_DIR_PIN, GPIO_OUT);
    gpio_put(PAN_MOTOR_DIR_PIN, 0);  /**< Default LOW, just be explicit */

    // gpio_init(TILT_MOTOR_DIR_PIN);
    // gpio_set_dir(TILT_MOTOR_DIR_PIN, GPIO_OUT);
    // gpio_put(TILT_MOTOR_DIR_PIN, 0);  /**< Default LOW, just be explicit */

    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);  /**< Default LOW, just be explicit */

    // // Magnet debugging
    // while (!AS5600_magnetGood(&panEncoder)) {
    //     printf("Magnet not right...\n");
    //     sleep_ms(1000);
    // }
    // printf("Magnet good!\n");

    printf("Starting timer\n");
    struct repeating_timer timer; // Maybe use real time clock peripheral if for use longer than ~72 mins
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
            cancel_repeating_timer(&timer);
            printf("Tunring interrupts off\n");
            uint32_t interrupts = save_and_disable_interrupts();
            for (int i = 0; i < 100000000; i++) {
                //
            }
            restore_interrupts(interrupts);
            printf("Tunring interrupts back on\n");
            add_repeating_timer_us(-CONTROLS_FREQ, updateMotors, NULL, &timer);
        }
    }

    return 0;
}
