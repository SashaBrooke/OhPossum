/**
 * @file gimbal_configuration.h
 * @brief Gimbal settings and configurations module.
 *
 * This file defines gimbal runtime settings and configurations saved to
 * non volatile flash memory. This module also provides the interface for
 * accessing and manipulating these parameters during gimbal operation.
 */

#ifndef GIMBAL_CONFIGURATION_H
#define GIMBAL_CONFIGURATION_H

#include "pid.h"
#include "as5600.h"

// Gimbal serial number is restricted to a uint8 for now
#define GIMBAL_SERIAL_NUMBER_MIN 0
#define GIMBAL_SERIAL_NUMBER_MAX 255

// Streaming rate range
extern const int GIMBAL_SLOW_STREAM_RATE;
extern const int GIMBAL_FAST_STREAM_RATE;

/**
 * @enum gimbal_mode_t
 * @brief Operational modes for the gimbal.
 */
typedef enum {
    GIMBAL_MODE_LOWER_LIMIT,
    GIMBAL_MODE_FREE,
    GIMBAL_MODE_ARMED,
    GIMBAL_MODE_UPPER_LIMIT
} gimbal_mode_t;

/**
 * @struct gimbal_t
 * @brief Represents the runtime state of the gimbal.
 */
typedef struct {
    bool savedConfiguration;

    gimbal_mode_t gimbalMode;

    float panPositionSetpoint;
    // float tiltPositionSetpoint;

    bool streaming;
    int streamRate;

    volatile PID_t panPositionController;
    // PID_t tiltPositionController;

    volatile AS5600_t panEncoder;
    // AS5600_t tiltEncoder;
} gimbal_t;

/**
 * @struct gimbal_configuration_t
 * @brief Represents the saved configuration of the gimbal.
 */
typedef struct {
    // Version tracking
    uint8_t serialNumber;

    // Controls gains
    volatile PID_t panPositionController;
    // volatile PID_t tiltPositionController;
} gimbal_configuration_t;

/**
 * @brief Initialises the gimbal runtime state.
 *
 * This function prepares the gimbal structure for operation by 
 * setting default values.
 *
 * @param gimbal Pointer to the gimbal structure to initialise.
 */
void setupGimbal(gimbal_t *gimbal);

/**
 * @brief Displays the current runtime state of the gimbal.
 * @param gimbal Pointer to the gimbal structure to display.
 */
void displayGimbal(gimbal_t *gimbal);

/**
 * @brief Loads the saved gimbal configuration.
 *
 * Retrieves the a saved gimbal configuration from non-volatile memory.
 *
 * @param config Pointer to the gimbal configuration structure to load into.
 */
void loadGimbalConfiguration(gimbal_configuration_t *config);

/**
 * @brief Displays the saved configuration of the gimbal.
 * @param config Pointer to the gimbal configuration structure to display.
 */
void displayGimbalConfiguration(gimbal_configuration_t *config);

/**
 * @brief Saves the current gimbal configuration.
 *
 * Stores the gimbal's configuration in non-volatile memory.
 *
 * @param config Pointer to the gimbal configuration structure to save.
 */
void saveGimbalConfiguration(gimbal_configuration_t *config);

#endif // GIMBAL_CONFIGURATION_H
