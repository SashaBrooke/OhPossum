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
#define GIMBAL_SERIAL_NUMBER_MIN  0
#define GIMBAL_SERIAL_NUMBER_MAX  255

// Gimbal range of motion (ROM)
#define GIMBAL_DEFAULT_UNSET_ROM  (-1)
#define GIMBAL_360_ROM            (-2)

// Streaming rate range
extern const int GIMBAL_SLOW_STREAM_RATE;
extern const int GIMBAL_FAST_STREAM_RATE;

/**
 * @enum gimbal_mode_e
 * @brief Operational modes for the gimbal.
 */
typedef enum {
    GIMBAL_MODE_LOWER_LIMIT,
    GIMBAL_MODE_FREE,
    GIMBAL_MODE_ARMED,
    GIMBAL_MODE_UPPER_LIMIT
} gimbal_mode_e;

/**
 * @struct gimbal_t
 * @brief Represents the runtime state of the gimbal.
 */
typedef struct {
    // Saved flag
    bool savedConfiguration;

    // Operational mode
    gimbal_mode_e gimbalMode;

    // Axis setpoints
    float panPositionSetpoint;

    // Axis soft limits
    float panLowerLimit;
    float panUpperLimit;

    // Global stream
    bool streaming;
    int streamRate;

    // Specific streams
    bool panPositionStream;
    bool panPidStream;
    bool panMotorStream;

    // Axis PID controllers
    volatile PID_t panPositionController;

    // Axis encoders
    volatile AS5600_t panEncoder;
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
 * @return `true` if a configuration was loaded,
 *         `false` if none was found to load.
 */
bool loadGimbalConfiguration(gimbal_configuration_t *config);

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

/**
 * @brief Calculates an XOR checksum for a given data buffer.
 *
 * This function iterates over each byte in the provided data buffer and calculates
 * the checksum by XORing each byte. The resulting checksum is returned as an 8-bit
 * value. This checksum can be used to verify the integrity of the data later.
 *
 * @param[in] data Pointer to the data buffer for which the checksum is calculated.
 * @param[in] size The size (in bytes) of the data buffer.
 *
 * @return The XOR checksum of the provided data buffer as an 8-bit unsigned integer.
 */
uint8_t calculateChecksum(uint8_t* data, int size);

/**
 * @brief Verifies the integrity of the data using the provided checksum.
 *
 * This function extracts the stored checksum from the last byte of the data buffer,
 * recalculates the checksum over the remaining data (excluding the checksum byte),
 * and compares the two. If the recalculated checksum matches the stored checksum,
 * the data is considered valid; otherwise, it is deemed corrupted/modified/invalid.
 *
 * @param[in] data Pointer to the data buffer containing both the data and the checksum.
 * @param[in] sizeWithChecksum The size (in bytes) of the entire data buffer, including the checksum.
 *
 * @return `true` if the checksum verification is successful (i.e., the checksums match),
 *         `false` if the checksum verification fails (i.e., the checksums do not match).
 */
bool verifyChecksum(uint8_t *data, int sizeWithChecksum);

#endif // GIMBAL_CONFIGURATION_H
