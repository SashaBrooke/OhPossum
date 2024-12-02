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

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0             
#define DIR_PIN 3                
#define I2C_SDA_PIN 4          
#define I2C_SCL_PIN 5          

static int addr = 0x36;         /**< I2C address of the AS5600 encoder (can only 
                                     be changed on the AS5600L variant) */

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

    encoder_init();

    while(true) {
        // Read angle from encoder
        uint8_t RAW_ANGLE_REG = 0x0C;       // Starts at the register for the high byte and increments 
                                            // once automatically to the register of the low byte
        uint8_t raw_angle[2];
        i2c_write_blocking(I2C_PORT, addr, &RAW_ANGLE_REG, 1, true);
        i2c_read_blocking(I2C_PORT, addr, raw_angle, 2, false);

        // Shift the lower 4 bits from the high byte and combine with the 8 bits from the low byte
        uint16_t combined_raw_angle = ((raw_angle[0] & 0x0F) << 8) | raw_angle[1];

        printf("%u\n", combined_raw_angle);

        sleep_ms(100);
    }

    return 0;
}
