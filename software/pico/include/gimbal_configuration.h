#ifndef GIMBAL_CONFIGURATION_H
#define GIMBAL_CONFIGURATION_H

#include "pid.h"
#include "as5600.h"

typedef enum {
    GIMBAL_MODE_FREE,
    GIMBAL_MODE_ARMED
} gimbal_mode_t;

typedef struct {
    // Version tracking
    uint8_t serialNumber;

    // Controls gains
    volatile PID_t panPositionController;
    // volatile PID_t tiltPositionController;
} gimbal_configuration_t;

void loadGimbalConfiguration(gimbal_configuration_t *config);

void saveGimbalConfiguration(gimbal_configuration_t *config);

#endif