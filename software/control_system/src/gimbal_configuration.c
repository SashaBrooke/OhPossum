/**
 * @file gimbal_configuration.c
 * @brief Source file for gimbal settings and configurations module.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "gimbal_configuration.h"

#define FLASH_TARGET_OFFSET (256 * 1024)  // Choosing to start at 256K

//  DEFAULT GIMBAL VALUES
const bool           GIMBAL_DEFAULT_SAVED_CONFIG_FLAG        = true;              // No new settings to save on setup
const gimbal_mode_e  GIMBAL_DEFAULT_SAFE_MODE                = GIMBAL_MODE_FREE;  // Safety: Force free mode on powerup
const float          GIMBAL_DEFAULT_AXIS_SETPOINT            = GIMBAL_DEFAULT_UNSET_ROM;
const float          GIMBAL_DEFAULT_AXIS_SOFT_LIMIT          = GIMBAL_DEFAULT_UNSET_ROM;
const bool           GIMBAL_DEFAULT_GLOBAL_STREAMING_STATUS  = true;
const int            GIMBAL_SLOW_STREAM_RATE                 = 10000;  
const int            GIMBAL_FAST_STREAM_RATE                 = 1;
const bool           GIMBAL_DEFAULT_SPECIFIC_STREAM_STATUS   = true;              // Must manually add desired packets

static gimbal_configuration_t gc;

/* Set up gimbal with default settings */
void setupGimbal(gimbal_t *gimbal) {
    gimbal->savedConfiguration  = GIMBAL_DEFAULT_SAVED_CONFIG_FLAG;

    gimbal->gimbalMode          = GIMBAL_DEFAULT_SAFE_MODE;

    gimbal->panPositionSetpoint = GIMBAL_DEFAULT_AXIS_SETPOINT;

    gimbal->panLowerLimit       = GIMBAL_DEFAULT_AXIS_SOFT_LIMIT;
    gimbal->panUpperLimit       = GIMBAL_DEFAULT_AXIS_SOFT_LIMIT;

    gimbal->streaming           = GIMBAL_DEFAULT_GLOBAL_STREAMING_STATUS;
    gimbal->streamRate          = GIMBAL_SLOW_STREAM_RATE; 

    gimbal->panPositionStream   = GIMBAL_DEFAULT_SPECIFIC_STREAM_STATUS;
    gimbal->panPidStream        = GIMBAL_DEFAULT_SPECIFIC_STREAM_STATUS;
    gimbal->panMotorStream      = GIMBAL_DEFAULT_SPECIFIC_STREAM_STATUS;
}

/* Display gimbal settings */
void displayGimbal(gimbal_t *gimbal) {
    printf("----------- Gimbal: -----------\n");
    
    // Whether the configuration is saved
    printf("Saved Configuration: %s\n", gimbal->savedConfiguration ? "Yes" : "No");

    // Gimbal mode
    if (gimbal->gimbalMode == GIMBAL_MODE_FREE) {
        printf("Gimbal Mode: GIMBAL_MODE_FREE\n");
    } else if (gimbal->gimbalMode == GIMBAL_MODE_ARMED) {
        printf("Gimbal Mode: GIMBAL_MODE_ARMED\n");
    } else {
        printf("Gimbal Mode: UNKNOWN\n");
    }

    // Pan position setpoint
    printf("Pan Position Setpoint: %.2f\n", gimbal->panPositionSetpoint);

    // Pan axis limits
    printf("Pan Lower Limit: %.2f\n", gimbal->panLowerLimit);
    printf("Pan Upper Limit: %.2f\n", gimbal->panUpperLimit);

    // Streaming status and rate
    printf("Streaming (global): %s\n", gimbal->streaming ? "Enabled" : "Disabled");
    printf("Stream Rate: %d\n", gimbal->streamRate);

    // Specific streams
    printf("Pan position stream: %s\n", gimbal->panPositionStream ? "Enabled" : "Disabled");
    printf("Pan PID stream: %s\n", gimbal->panPidStream ? "Enabled" : "Disabled");
    printf("Pan motor stream: %s\n", gimbal->panMotorStream ? "Enabled" : "Disabled");

    // Pan position controller PID values
    printf("Pan Position Controller:\n");
    printf("  Kp: %.2f\n", gimbal->panPositionController.Kp);
    printf("  Ki: %.2f\n", gimbal->panPositionController.Ki);
    printf("  Kd: %.2f\n", gimbal->panPositionController.Kd);
    printf("  tau: %.2f\n", gimbal->panPositionController.tau);
    printf("  outLimMin: %.2f\n", gimbal->panPositionController.outLimMin);
    printf("  outLimMax: %.2f\n", gimbal->panPositionController.outLimMax);
    printf("  intLimMin: %.2f\n", gimbal->panPositionController.intLimMin);
    printf("  intLimMax: %.2f\n", gimbal->panPositionController.intLimMax);
    printf("  T: %.4f\n", gimbal->panPositionController.T);
    printf("  integrator: %.2f\n", gimbal->panPositionController.integrator);
    printf("  prevError: %.2f\n", gimbal->panPositionController.prevError);
    printf("  differentiator: %.2f\n", gimbal->panPositionController.differentiator);
    printf("  prevMeasurement: %.2f\n", gimbal->panPositionController.prevMeasurement);
    printf("  output: %.2f\n", gimbal->panPositionController.output);
    printf("  maxMeasurement: %.2f\n", gimbal->panPositionController.maxMeasurement);

    // Pan encoder
    printf("Pan Encoder:\n");
    printf("  Initialised: %s\n", gimbal->panEncoder.initialised ? "Yes" : "No");
    printf("  DIR_PIN: %d\n", gimbal->panEncoder.DIR_PIN);
    printf("  Offset: %u\n", gimbal->panEncoder.offset);

    printf("\n");
}

/* Load gimbal configuration from non-volatile flash memory */
bool loadGimbalConfiguration(gimbal_configuration_t *config) {
    if (config == NULL) {
        return false;
    }

    // Calculate the address in flash memory
    const uint8_t* flash_target_contents = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);

    // Create a buffer to hold the configuration data and the checksum
    int configSize = sizeof(gimbal_configuration_t);
    int bufferSize = configSize + 1;
    uint8_t *buffer = malloc(bufferSize);

    memcpy(buffer, flash_target_contents, bufferSize);

    if (verifyChecksum(buffer, bufferSize)) {
        // If the checksum is valid, copy the saved configuration into the provided structure
        memcpy(config, buffer, configSize);
        free(buffer);
        printf("Gimbal configuration loaded successfully.\n");
        return true;
    } else {
        free(buffer);
        printf("No configuration to load.\n");
        return false;
    }
}

/* Display gimbal configuration */
void displayGimbalConfiguration(gimbal_configuration_t *config) {
    printf("---- Gimbal configuration: ----\n");

    // Gimbal serial number
    printf("Gimbal Serial Number: %d\n", config->serialNumber);

    // Pan position controller PID values
    printf("Pan Position Controller:\n");
    printf("  Kp: %.2f\n", config->panPositionController.Kp);
    printf("  Ki: %.2f\n", config->panPositionController.Ki);
    printf("  Kd: %.2f\n", config->panPositionController.Kd);
    printf("  tau: %.2f\n", config->panPositionController.tau);
    printf("  outLimMin: %.2f\n", config->panPositionController.outLimMin);
    printf("  outLimMax: %.2f\n", config->panPositionController.outLimMax);
    printf("  intLimMin: %.2f\n", config->panPositionController.intLimMin);
    printf("  intLimMax: %.2f\n", config->panPositionController.intLimMax);
    printf("  T: %.4f\n", config->panPositionController.T);
    printf("  integrator: %.2f\n", config->panPositionController.integrator);
    printf("  prevError: %.2f\n", config->panPositionController.prevError);
    printf("  differentiator: %.2f\n", config->panPositionController.differentiator);
    printf("  prevMeasurement: %.2f\n", config->panPositionController.prevMeasurement);
    printf("  output: %.2f\n", config->panPositionController.output);
    printf("  maxMeasurement: %.2f\n", config->panPositionController.maxMeasurement);

    printf("\n");
}

/* Save gimbal configuration to non-volatile flash memory */
void saveGimbalConfiguration(gimbal_configuration_t *config) {
    memcpy(&gc, config, sizeof(gimbal_configuration_t));
    uint8_t* configAsBytes = (uint8_t*) &gc;
    int configSize = sizeof(*config);

    // Create a buffer to hold the configuration and a computed checksum
    int bufferSize = configSize + 1;
    uint8_t *buffer = malloc(bufferSize);
    memcpy(buffer, configAsBytes, configSize);

    uint8_t checksum = calculateChecksum(configAsBytes, configSize);
    buffer[configSize] = checksum;  // Append the checksum as the last byte
    
    // Calculate how many flash pages we're gonna need to write
    int writeSize = (bufferSize / FLASH_PAGE_SIZE) + 1;

    // Calculate how many flash sectors we're gonna need to erase
    int sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1;
        
    printf("Programming flash target region...\n");

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE * writeSize);
    restore_interrupts(interrupts);

    free(buffer);

    printf("Done.\n");
}

/* Calculate XOR checksum */
uint8_t calculateChecksum(uint8_t* data, int size) {
    uint8_t checksum = 0;
    for (int i = 0; i < size; i++) {
        checksum ^= data[i];  // XOR checksum for simplicity
    }
    return checksum;
}

/* Verify data (wth an included checksum) integrity */
bool verifyChecksum(uint8_t *data, int sizeWithChecksum) {
    int configSize = sizeWithChecksum - 1;  // Exclude the checksum byte
    uint8_t storedChecksum = data[configSize];  // Extract stored checksum
    uint8_t calculatedChecksum = calculateChecksum(data, configSize);  // Recalculate checksum

    return (storedChecksum == calculatedChecksum);
}
