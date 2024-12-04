// TODO: Documentation

#ifndef AS5600_H
#define AS5600_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Address
const uint8_t AS5600_DEFAULT_I2C_ADDR   = 0x36;

//  Directions
const uint8_t AS5600_CLOCK_WISE         = 0;  //  LOW
const uint8_t AS5600_COUNTERCLOCK_WISE  = 1;  //  HIGH

//  Conversions
const float   AS5600_RAW_TO_DEGREES     = 360.0 / 4096;
const float   AS5600_DEGREES_TO_RAW     = 4096 / 360.0;

AS5600_t *AS5600_setup(i2c_inst_t *i2c, uint8_t DIR_PIN, uint8_t direction);

bool AS5600_isConnected(AS5600_t *enc);

uint8_t AS5600_readStatus(AS5600_t *enc);

bool AS5600_magnetDetected(AS5600_t *enc);

bool AS5600_magnetTooWeak(AS5600_t *enc);

bool AS5600_magnetTooStrong(AS5600_t *enc);

bool AS5600_magnetGood(AS5600_t *enc);

uint8_t AS5600_readAGC(AS5600_t *enc);

uint16_t AS5600_getRawAngle(AS5600_t *enc);

#endif