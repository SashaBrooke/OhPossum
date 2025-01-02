// TODO: Documentation

#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>

#include "hardware/i2c.h"

// Address
extern const uint8_t   AS5600_DEFAULT_I2C_ADDR;

//  Directions
extern const uint8_t   AS5600_CLOCK_WISE;
extern const uint8_t   AS5600_COUNTERCLOCK_WISE;

//  Conversions
extern const float     AS5600_RAW_TO_DEGREES;
extern const float     AS5600_DEGREES_TO_RAW;

//  Resolution
extern const uint16_t  AS5600_RAW_ANGLE_MIN;
extern const uint16_t  AS5600_RAW_ANGLE_MAX;

typedef struct {
    // Initialisation
    bool initialised;

    // Hardware
    i2c_inst_t *i2c;
    uint8_t DIR_PIN;

    // Offset
    uint16_t offset;
} AS5600_t;

AS5600_t AS5600_setup(i2c_inst_t *i2c, uint8_t DIR_PIN, uint8_t direction);

bool AS5600_isConnected(volatile AS5600_t *enc);

uint8_t AS5600_readStatus(volatile AS5600_t *enc);

bool AS5600_magnetDetected(volatile AS5600_t *enc);

bool AS5600_magnetTooWeak(volatile AS5600_t *enc);

bool AS5600_magnetTooStrong(volatile AS5600_t *enc);

bool AS5600_magnetGood(volatile AS5600_t *enc);

uint8_t AS5600_readAGC(volatile AS5600_t *enc);

uint16_t AS5600_getRawAngle(volatile AS5600_t *enc);

#endif