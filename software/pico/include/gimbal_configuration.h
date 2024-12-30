#ifndef GIMBAL_CONFIGURATION_H
#define GIMBAL_CONFIGURATION_H

#include "pid.h"
#include "as5600.h"

typedef enum {
    GIMBAL_MODE_LOWER_LIMIT,
    GIMBAL_MODE_FREE,
    GIMBAL_MODE_ARMED,
    GIMBAL_MODE_UPPER_LIMIT
} gimbal_mode_t;

typedef struct {
    bool savedConfiguration;

    gimbal_mode_t gimbalMode;

    float panPositionSetpoint;
    // float tiltPositionSetpoint;

    bool streaming;
    int streamRate;

    PID_t panPositionController;
    // PID_t tiltPositionController;

    AS5600_t panEncoder;
    // AS5600_t tiltEncoder;
} gimbal_t;

typedef struct {
    // Version tracking
    uint8_t serialNumber;

    // Controls gains
    volatile PID_t panPositionController;
    // volatile PID_t tiltPositionController;
} gimbal_configuration_t;

void setupGimbal(gimbal_t *gimbal);

void displayGimbal(gimbal_t *gimbal);

void loadGimbalConfiguration(gimbal_configuration_t *config);

void displayGimbalConfiguration(gimbal_configuration_t *config);

void saveGimbalConfiguration(gimbal_configuration_t *config);

#endif