// TODO: Documentation
// Currently supports CR, LF and CRLF end of message
// Currently only supports a single input stream
// Currently limits the input buffer to a static length of 100

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#include "pico/stdlib.h"

#include "command.h"
#include "pid.h"
#include "as5600.h"
#include "gimbal_configuration.h"

#define LF           10
#define CR           13
#define NO_VAL       254
#define ENDSTDIN     255

#define DECIMAL_BASE 10

static char strg[100];
static int lp = 0;

void resetSerialCommandInput() {
    memset(strg, 0, sizeof(strg));
    lp = 0;
}

// Designed to be ran inside a repeating loop. Each loop iteration this
// function will instantly read a single character without holding up
// the execution of the program. Once the full message is read (CR, LF,
// CRLF encountered), it is returned.
// Input must be reset using resetSerialCommandInput() after each read.
char *readSerialCommand_nonBlocking() {
    char chr = getchar_timeout_us(0);

    // Skip if nothing to read (invalid) or error occurred
    if (chr == NO_VAL || chr == ENDSTDIN) {
        return NULL;
    }

    // Only process valid ASCII characters, CR, or LF
    if ((chr >= 32 && chr <= 126)) {
        strg[lp++] = chr;
    }

    bool endOfInput = (chr == CR && (getchar_timeout_us(0) == LF)) || 
                        (chr == CR) || 
                        (chr == LF);

    bool bufferFull = (lp == (sizeof(strg) - 1));

    // Terminate string on CR, LF, or CRLF or buffer full
    if (endOfInput || bufferFull) {
        strg[lp] = 0;  // Null-terminate
        if (strlen(strg) > 0) {
            return strg;
        }
    }

    return NULL;
}

void processCommands(char *input, gimbal_t *gimbal, gimbal_configuration_t *config) {
    char *command = strtok(input, " ");
    while (command != NULL) {
        executeCommand(command, gimbal, config);
        command = strtok(NULL, " ");
    }
}

void executeCommand(char *command, gimbal_t *gimbal, gimbal_configuration_t *config) {
    if (command == NULL || strlen(command) == 0) {
        printf("Empty command encountered.\n");
        return;
    }

    char *equals = strchr(command, '=');
    char *name = command;
    char *valueStr = NULL;

    if (equals) {
        *equals = '\0';
        valueStr = equals + 1;
    }

    // ####################################################################################
    //                               AVAILABLE COMMANDS
    // ####################################################################################
    command_t commands[] = {
        {"help", "Show available commands."},
        {"config-read", "Display the current gimbal configuration."},
        {"config-save", "Save the current gimbal configuration."},
        {"gimbal-read", "Display the current state of the gimbal."},
        {"gimbal-free", "Set the gimbal to free mode."},
        {"gimbal-arm", "Arm the gimbal and set initial pan and tilt setpoints."},
        {"gimbal-stream", "Enable or disable gimbal streaming."},
        {"gimbal-streamrate", "Set the gimbal stream rate."},
        {"gimbal-serialno", "Set the gimbal serial number (limited to uint8 values)."},
        {"pan-setpos", "Set the pan setpoint."}
        // pan pid commands
        // pan encoder offset (offset implementation TODO)
        // tilt-setpos
        // tilt pid commands
        // tilt encoder offset (offset implementation TODO)
    };

    if (strcmp(name, "help") == 0) { // "h" shortcut
        size_t numCommands = sizeof(commands) / sizeof(command_t);
        const int nameWidth = 20;

        printf("Available commands:\n");
        for (size_t i = 0; i < numCommands; ++i) {
            printf("  %-*s %s\n", nameWidth, commands[i].name, commands[i].description);
        }
    }

    else if (strcmp(name, "config-read") == 0) { // "cr" shortcut
        displayGimbalConfiguration(config);
    } 
    
    else if (strcmp(name, "config-save") == 0) { // "cs" shortcut
        saveGimbalConfiguration(config);
        gimbal->savedConfiguration = true;
    } 
    
    else if (strcmp(name, "gimbal-read") == 0) { // "gr" shortcut
        displayGimbal(gimbal);
    } 

    else if (strcmp(name, "gimbal-free") == 0) { // "free" shortcut
        gimbal->gimbalMode = GIMBAL_MODE_FREE;
        printf("Gimbal disarmed\n");
    }

    else if (strcmp(name, "gimbal-arm") == 0) { // "arm" shortcut
        uint16_t panRawAngle = AS5600_getRawAngle(&gimbal->panEncoder);
        // tilt

        gimbal->panPositionSetpoint = (float)panRawAngle;
        // tilt

        gimbal->gimbalMode = GIMBAL_MODE_ARMED;
        printf("Gimbal armed\n");
    } 

    else if (strcmp(name, "gimbal-stream") == 0) { // "gs" shortcut
        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
                printf("Invalid value for 'gimbal-stream'. Use 0 (false) or 1 (true).\n");
            } else {
                gimbal->streaming = (bool)value;
                printf("%s streaming\n", value ? "Enabled" : "Disabled");
            }
        } else {
            printf("Command 'gimbal-stream' requires a value.\n");
        }
    }

    else if (strcmp(name, "gimbal-streamrate") == 0) { // "gsr" shortcut
        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || value < GIMBAL_FAST_STREAM_RATE || value > GIMBAL_SLOW_STREAM_RATE) {
                printf("Invalid value for 'gimbal-streamrate'. Use a value between %d-%d.\n",
                    GIMBAL_FAST_STREAM_RATE, GIMBAL_SLOW_STREAM_RATE);
            } else {
                gimbal->streamRate = (int)value;
                printf("Updated stream rate to %d\n", (int)value);
            }
        } else {
            printf("Command 'gimbal-streamrate' requires a value.\n");
        }
    }

    else if (strcmp(name, "gimbal-serialno") == 0) { // "gsn" shortcut
        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || value < GIMBAL_SERIAL_NUMBER_MIN || value > GIMBAL_SERIAL_NUMBER_MAX) {
                printf("Invalid value for 'gimbal-serialno'. Must be between %d-%d.\n",
                       GIMBAL_SERIAL_NUMBER_MIN, GIMBAL_SERIAL_NUMBER_MAX);
            } else {
                config->serialNumber = (int)value;
                gimbal->savedConfiguration = false;
                printf("Updated gimbal serial number to %d\n", (int)value);
            }
        } else {
            printf("Command 'gimbal-serialno' requires a value.\n");
        }
    }

    else if (strcmp(name, "pan-setpos") == 0) { // "ps" shortcut
    if (valueStr && valueStr[0] != '\0') {
        char *endptr;
        errno = 0;

        float value = strtof(valueStr, &endptr);

        if (errno != 0 || *endptr != '\0' || value < AS5600_RAW_ANGLE_MIN || value > AS5600_RAW_ANGLE_MAX) {
            printf("Invalid value for 'pan-setpos'. Setpoint values must be between %u-%u.\n",
                   AS5600_RAW_ANGLE_MIN, AS5600_RAW_ANGLE_MAX);
        } else {
            gimbal->panPositionSetpoint = value;
            printf("Updated pan setpoint to %f\n", value);
        }
    } else {
        printf("Command 'pan-setpos' requires a value.\n");
    }
}

    // pan pid commands

    // pan encoder offset (offset implementation TODO)

    // tilt-setpos

    // tilt pid commands

    // tilt encoder offset (offset implementation TODO)
    
    else {
        printf("Unknown command: '%s'\n", name);
    }
}
