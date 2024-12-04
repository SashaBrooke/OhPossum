#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "AS5600.h"

//  OUTPUT REGISTERS
const uint8_t AS5600_RAW_ANGLE_REG     = 0x0C;  // + 0x0D

//  STATUS REGISTERS
const uint8_t AS5600_STATUS_REG        = 0x0B;
const uint8_t AS5600_AGC_REG           = 0x1A;

//  STATUS BITS
const uint8_t AS5600_MAGNET_DETECTED   = 0x20;
const uint8_t AS5600_MAGNET_TOO_WEAK   = 0x10;
const uint8_t AS5600_MAGNET_TOO_STRONG = 0x08;

void AS5600_init(AS5600_t *enc, i2c_inst_t *i2c, uint8_t DIR_PIN, uint8_t direction) {
    enc->i2c = i2c;

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, direction);

    enc->initialised = true;
}

bool AS5600_isConnected(AS5600_t *enc) {
    if (enc->initialised) {
        int result = i2c_write_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, NULL, 0, false);
        return result >= 0; // If result is >= 0, the device acknowledged the address
    }
    return false;
}

// TODO: Read magnet status

uint16_t AS5600_getRawAngle(AS5600_t *enc) {
    if (enc->initialised) {
        uint8_t rawAngleReg = AS5600_RAW_ANGLE_REG;
        uint8_t rawAngle[2];

        i2c_write_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, &rawAngleReg, 1, true);
        i2c_read_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, rawAngle, 2, false);

        // Shift the lower 4 bits from the high byte and combine with the 8 bits from the low byte
        combinedRawAngle = ((rawAngle[0] & 0x0F) << 8) | rawAngle[1];

        return combinedRawAngle;
    }
    return 0;
}
