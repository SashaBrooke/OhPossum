/**
 * @file encoder.c
 * @brief Prototyping code for reading raw angle position from an AS5600 magnetic encoder
 *
 * This program initializes I2C communication with the AS5600 magnetic encoder, checks the magnet
 * status (whether it is detected, too weak, or too strong), and continuously reads the raw angle 
 * data from the encoder.
 *
 * The program outputs the combined raw angle value to the console, which can be further processed
 * for position tracking.
 */

#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "PID.h"

#define I2C_PORT i2c0
#define DIR_PIN 3
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define TEST_PIN 0

typedef struct repeating_timer repeating_timer_t;

static int addr = 0x36;         /**< I2C address of the AS5600 encoder (can only 
                                     be changed on the AS5600L variant) */

volatile uint16_t combined_raw_angle = 0;
volatile bool print_flag = false;
volatile PID_t positionController;
volatile float pid_output = 0.0f;

/**
 * @brief Initializes the encoder and checks the magnet's status.
 * 
 * This function ensures the presence of a magnet and verifies that its strength is within 
 * the optimal range for reliable measurements. It repeatedly checks the magnet status by 
 * reading the STATUS register of the encoder.
 *
 * - If no magnet is detected, it will print a message and retry.
 * - If the magnet is detected but too weak or too strong, it will print a message and retry.
 * - If the magnet strength is within the optimal range, initialization is complete and proceed.
 * 
 * @note The function will wait indefinitely until the magnet status is confirmed to be within 
 * the optimal range once and will then move on.
 */
void encoder_init() {
    bool magnet_good = false;
    while (!magnet_good) {
        uint8_t STATUS_REG = 0x0B;
        uint8_t magnet_status[1];

        i2c_write_blocking(I2C_PORT, addr, &STATUS_REG, 1, true);
        i2c_read_blocking(I2C_PORT, addr, magnet_status, 1, false);

        uint8_t status = magnet_status[0];
        uint8_t md = (status & 0x20) >> 5;  /**< Magnet Detected (bit 5) */
        uint8_t ml = (status & 0x10) >> 4;  /**< Magnet Too Weak (bit 4) */
        uint8_t mh = (status & 0x08) >> 3;  /**< Magnet Too Strong (bit 3) */

        if (md) {
            printf("Magnet detected.\n");
            if (ml) {
                printf("Magnet too weak.\n");
            }
            if (mh) {
                printf("Magnet too strong.\n");
            }
            if (!ml && !mh) {
                printf("Magnet strength is within optimal range.\n");
                magnet_good = true;  /**< Proceed if magnet strength is optimal */
            }
        } else {
            printf("No magnet detected.\n");
        }
        sleep_ms(1000);  /**< Wait before retrying if magnet is not optimal */
    }
}

/**
 * @brief Reads the raw angle data from the AS5600 magnetic encoder and calculates a PID output.
 *
 * This function is a callback for a repeating timer. It communicates with the AS5600 encoder over I2C
 * to read the raw angle value and calculates a PID response. Every 100 calls, it sets a flag 
 * (`print_flag`) to  * signal the main loop to print the angle value. The function also toggles a GPIO 
 * pin (`TEST_PIN`) for debugging purposes to measure timing.
 *
 * @param timer Pointer to the repeating timer structure. Unused in this function.
 * @return true Always returns true to keep the timer active.
 * 
 * @note Encoder raw angle read and PID calculation takes approximately 150us. Could probably only push
 *       timer to minimum 200us between repetitions to be safe.
 */
bool read_raw_angle(repeating_timer_t *timer) {
    gpio_put(TEST_PIN, 1); // Pin toggle for debug

    uint8_t RAW_ANGLE_REG = 0x0C;       // Starts at the register for the high byte and increments 
                                        // once automatically to the register of the low byte
    uint8_t raw_angle[2];
    i2c_write_blocking(I2C_PORT, addr, &RAW_ANGLE_REG, 1, true);
    i2c_read_blocking(I2C_PORT, addr, raw_angle, 2, false);

    // Shift the lower 4 bits from the high byte and combine with the 8 bits from the low byte
    combined_raw_angle = ((raw_angle[0] & 0x0F) << 8) | raw_angle[1];

    // Calculate PID response demanding a setpoint raw angle of 1000
    pid_output = PID_Update(&positionController, 1000.0f, (float)combined_raw_angle);

    // Track the number of calls to this function and print at multiples of 100
    static int counter = 0; 
    counter++;
    if (counter >= 100) {
        print_flag = true;
        counter = 0;
    }

    gpio_put(TEST_PIN, 0); // Pin toggle for debug

    return true;
}

int main() {
    stdio_init_all();

    sleep_ms(10000); // Allow time to open serial monitor

    // Configure I2C Communication
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 0);  /**< Direction set LOW = clockwise */

    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);  /**< Default LOW, just be explicit */

    encoder_init();

    PID_Init(&positionController, 1.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 0.0f, 0.0f, 0.0015f);

    struct repeating_timer timer; // Maybe use real time clock peripheral if for use longer than ~72 mins
    add_repeating_timer_us(-1500, read_raw_angle, NULL, &timer);

    while(true) {
        if (print_flag) {
            printf("Raw angle: [%u]\tPID Output: [%f]\n", combined_raw_angle, pid_output);
            print_flag = false;  /**< Reset print flag */
        }
    }

    return 0;
}
