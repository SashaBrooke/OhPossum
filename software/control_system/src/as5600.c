/**
 * @file as5600.c
 * @brief Source file for the AS5600 magnetic rotary encoder library module.
 */

#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "as5600.h"

// Address
const uint8_t  AS5600_DEFAULT_I2C_ADDR     = 0x36;

//  Directions
const uint8_t  AS5600_CLOCK_WISE           = 0;  //  LOW
const uint8_t  AS5600_COUNTERCLOCK_WISE    = 1;  //  HIGH

//  Conversions
const float    AS5600_RAW_TO_DEGREES       = 360.0 / 4096;
const float    AS5600_DEGREES_TO_RAW       = 4096 / 360.0;

//  Resolution
const uint16_t AS5600_ANGULAR_RESOLUTION   = 4096;  // 12-bit
const uint16_t AS5600_RAW_ANGLE_MIN        = 0;
const uint16_t AS5600_RAW_ANGLE_MAX        = 4095;

//  OUTPUT REGISTERS
const uint8_t  AS5600_RAW_ANGLE_REG        = 0x0C;  // + 0x0D

//  STATUS REGISTERS
const uint8_t  AS5600_STATUS_REG           = 0x0B;
const uint8_t  AS5600_AGC_REG              = 0x1A;

//  STATUS BITS
const uint8_t  AS5600_MAGNET_DETECTED      = 0x20;
const uint8_t  AS5600_MAGNET_TOO_WEAK      = 0x10;
const uint8_t  AS5600_MAGNET_TOO_STRONG    = 0x08;

/* Set up AS5600 encoder */
AS5600_t AS5600_setup(i2c_inst_t *i2c, uint8_t DIR_PIN, uint8_t direction) {
    AS5600_t encoder = {
        .initialised = true,
        .i2c = i2c,
        .DIR_PIN = DIR_PIN,
        .offset = 0
    };

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, direction);

    return encoder;
}

/* Check if the encoder is connected (reachable) */
bool AS5600_isConnected(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return false;
    }
    if (enc->initialised) {
        int result = i2c_write_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, NULL, 0, false);
        return result >= 0; // If result is >= 0, the device acknowledged the address
    }
    return false;
}

/* Read the encoder status register */
uint8_t AS5600_readStatus(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return 0;
    }
    if (enc->initialised) {
        uint8_t statusReg= AS5600_STATUS_REG;
        uint8_t magnetStatus[1];

        i2c_write_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, &statusReg, 1, true);
        i2c_read_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, magnetStatus, 1, false);

        uint8_t status = magnetStatus[0];

        return status;
    }
    return 0;
}

/* Check if the encoder magnet is detected */
bool AS5600_magnetDetected(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return false;
    }
    if (enc->initialised) {
        uint8_t status = AS5600_readStatus(enc);
        uint8_t md = (status & AS5600_MAGNET_DETECTED) >> 5;  /**< Magnet Detected (bit 5) */
        return (md == 1);
    }
    return false;
}

/* Check if the encoder magnet is too weak */
bool AS5600_magnetTooWeak(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return false;
    }
    if (enc->initialised) {
        uint8_t status = AS5600_readStatus(enc);
        uint8_t ml = (status & AS5600_MAGNET_TOO_WEAK) >> 4;  /**< Magnet Too Weak (bit 4) */
        return (ml == 1);
    }
    return false;
}

/* Check if the encoder magnet is too strong */
bool AS5600_magnetTooStrong(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return false;
    }
    if (enc->initialised) {
        uint8_t status = AS5600_readStatus(enc);
        uint8_t mh = (status & AS5600_MAGNET_TOO_STRONG) >> 3;  /**< Magnet Too Strong (bit 3) */
        return (mh == 1);
    }
    return false;
}

/* Check if the encoder magnet is within the optimal range */
bool AS5600_magnetGood(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return false;
    }
    if (enc->initialised) {
        uint8_t status = AS5600_readStatus(enc);
        uint8_t md = (status & 0x20) >> 5;  /**< Magnet Detected (bit 5) */
        uint8_t ml = (status & 0x10) >> 4;  /**< Magnet Too Weak (bit 4) */
        uint8_t mh = (status & 0x08) >> 3;  /**< Magnet Too Strong (bit 3) */
        return (md == 1) && (ml == 0) && (mh == 0);
    }
    return false;
}

/* Read the encoder AGC register */
uint8_t AS5600_readAGC(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return 0;
    }
    if (enc->initialised) {
        uint8_t agcReg= AS5600_AGC_REG;
        uint8_t agcValue[1];

        i2c_write_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, &agcReg, 1, true);
        i2c_read_blocking(enc->i2c, AS5600_DEFAULT_I2C_ADDR, agcValue, 1, false);

        uint8_t agc = agcValue[0];

        return agc;
    }
    return 0;
}

/* Read the encoder raw angle */
uint16_t AS5600_getRawAngle(volatile AS5600_t *enc) {
    if (enc == NULL) {
        return 0;
    }
    if (enc->initialised) {
        uint8_t rawAngleReg = AS5600_RAW_ANGLE_REG;
        uint8_t rawAngle[2];

        i2c_write_blocking(i2c0, AS5600_DEFAULT_I2C_ADDR, &rawAngleReg, 1, true);
        i2c_read_blocking(i2c0, AS5600_DEFAULT_I2C_ADDR, rawAngle, 2, false);

        // Shift the lower 4 bits from the high byte and combine with the 8 bits from the low byte
        uint16_t combinedRawAngle = ((rawAngle[0] & 0x0F) << 8) | rawAngle[1];

        return combinedRawAngle;
    }
    return 0;
}
