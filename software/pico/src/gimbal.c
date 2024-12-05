#include <stdio.h>

#include "PID.h"
#include "AS5600.h"

#define PAN_I2C_PORT i2c0
#define PAN_DIR_PIN 3
#define PAN_I2C_SDA_PIN 4
#define PAN_I2C_SCL_PIN 5

// #define TILT_I2C_PORT i2c1
// #define TILT_DIR_PIN XXX
// #define TILT_I2C_SDA_PIN XXX
// #define TILT_I2C_SCL_PIN XXX

#define TEST_PIN 0

typedef struct repeating_timer repeating_timer_t;

typedef struct {
    // Encoders
    AS5600_t *panEncoder;
    // AS5600_t *tiltEncoder;

    // PID Controllers
    PID_t *panPositionController;
    // PID_t *tiltPositionController;

    // PID_t *panRateController;
    // PID_t *tiltRateController;
} gimbal_t;

volatile uint16_t panRawAngle = 0;
volatile float panPidOutput = 0.0f;
volatile bool printFlag = false;

bool updateMotors(repeating_timer_t *timer) {
    volatile gimbal_t *gimbal = (gimbal_t *)timer->user_data;
    uint16_t rawAngle = AS5600_getRawAngle(gimbal->panEncoder);

    // PID

    // Set volatile global variables and handle printFlag
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

    volatile AS5600_t *panEncoder = AS5600_setup(PAN_I2C_PORT, PAN_DIR_PIN, AS5600_CLOCK_WISE);
    // volatile AS5600_t *tiltEncoder = AS5600_setup(TILT_I2C_PORT, TILT_DIR_PIN, AS5600_CLOCK_WISE);

    volatile PID_t *panPositionController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);
    // volatile PID_t *tiltPositionController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);

    // volatile PID_t *panRateController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);
    // volatile PID_t *tiltRateController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);

    volatile gimbal_t gimbal = {panEncoder, panPositionController};

    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);  /**< Default LOW, just be explicit */

    while (!AS5600_magnetGood(panEncoder)) {
        sleep_ms(1000);
    }

    struct repeating_timer timer; // Maybe use real time clock peripheral if for use longer than ~72 mins
    add_repeating_timer_us(-1500, updateMotors, &gimbal, &timer);

    while(true) {
        if (printFlag) {
            printf("Raw angle: [%u]\tPID Output: [%f]\n", panRawAngle, panPidOutput);
            printFlag = false;  /**< Reset print flag */
        }
    }

    // Free dynamically allocated variables
    free(panEncoder);
    // free(tiltEncoder);

    free(panPositionController);
    // free(tiltPositionController);

    // free(panRateController);
    // free(tiltRateController);

    return 0;
}