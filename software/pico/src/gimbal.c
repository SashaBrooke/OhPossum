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

// Encoders
volatile AS5600_t panEncoder;
// volatile AS5600_t tiltEncoder;

// PID Controllers
volatile PID_t panPositionController;
// volatile PID_t tiltPositionController;

// volatile PID_t panRateController;
// volatile PID_t tiltRateController;

volatile uint16_t panPos = 0;
volatile float panPid = 1.0f;
volatile bool printFlag = false;

bool updateMotors(repeating_timer_t *timer) {
    // Timing pin
    gpio_put(TEST_PIN, 1);

    // Read encoder
    uint16_t panRawAngle = AS5600_getRawAngle(&panEncoder);

    // PID
    float panPidOutput = PID_update(&panPositionController, 1000.0f, (float)panRawAngle);

    // Set volatile global variables and handle printFlag
    panPos = panRawAngle;
    panPid = panPidOutput;

    static int counter = 0; 
    counter++;
    if (counter >= 100) {
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

    panEncoder = AS5600_setup(PAN_I2C_PORT, PAN_DIR_PIN, AS5600_CLOCK_WISE);
    // tiltEncoder = AS5600_setup(TILT_I2C_PORT, TILT_DIR_PIN, AS5600_CLOCK_WISE);

    panPositionController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);
    // tiltPositionController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);

    // panRateController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);
    // tiltRateController = PID_setup(1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);

    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);  /**< Default LOW, just be explicit */

    while (!AS5600_magnetGood(&panEncoder)) {
        printf("Magnet not right...\n");
        sleep_ms(1000);
    }
    printf("Magnet good!\n");

    printf("Starting timer\n");
    struct repeating_timer timer; // Maybe use real time clock peripheral if for use longer than ~72 mins
    add_repeating_timer_us(-1500, updateMotors, NULL, &timer);

    while(true) {
        if (printFlag) {
            printf("Raw angle: [%u]\tPID Output: [%f]\n", panPos, panPid);
            // printFlag = false;  /**< Reset print flag */
        }
    }

    return 0;
}