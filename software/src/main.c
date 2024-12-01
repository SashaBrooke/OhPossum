/**
 * @file main.c
 * @brief Prototyping code to test the repeating timer module on Raspberry Pi Pico W.
 * 
 * Uses the repeating timer module to generate a repeating square wave switching
 * between high and low every 1500 us. The square wave is output on the pin defined 
 * by the TEST_PIN macro.
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

#define TEST_PIN 0 // GPIO pin number to output the square wave

volatile bool pin_on = false; // Tracks the current state of the pin (high or low)

/**
 * @brief Used to toggle a pin high and low.
 * 
 * This function is called by the repeating timer callback. If the pin is currently high,
 * it is toggled low. If the pin is currently low, it is toggled high.
 *
 * @param[in] timer Pointer to the repeating timer structure (not used in this implementation).
 * @retval true Indicates successful toggling of the pin.
 */
bool repeating_timer_callback(struct repeating_timer *timer) {
    if (pin_on == false) {
        gpio_put(TEST_PIN, 1);
        pin_on = true;
    } else if (pin_on == true) {
        gpio_put(TEST_PIN, 0);
        pin_on = false;
    }
    return true;
}

int main() {
    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0); // Don't assume low initially

    struct repeating_timer timer; // Consider using real time clock peripheral for use longer than ~72 mins
    add_repeating_timer_us(-1500, repeating_timer_callback, NULL, &timer);

    while(true) {
        // Empty
    }

    return 0;
}