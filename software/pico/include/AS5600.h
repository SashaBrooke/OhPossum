#ifndef AS5600_H
#define AS5600_H

#include "pico/stdlib.h"

// Address
const uint8_t AS5600_DEFAULT_I2C_ADDR   = 0x36;

//  Directions
const uint8_t AS5600_CLOCK_WISE         = 0;  //  LOW
const uint8_t AS5600_COUNTERCLOCK_WISE  = 1;  //  HIGH

//  Conversions
const float   AS5600_RAW_TO_DEGREES     = 360.0 / 4096;
const float   AS5600_DEGREES_TO_RAW     = 4096 / 360.0;

typedef struct {
    // Initialisation
    bool initialised;
    i2c_inst_t *i2c;

    // Offset
    uint16_t offset;
} AS5600_t;

void AS5600_init(AS5600_t *enc, i2c_inst_t *i2c, uint8_t DIR_PIN, uint8_t direction);

bool AS5600_isConnected(AS5600_t *enc);

float AS5600_getRawAngle(AS5600_t *enc);

#endif