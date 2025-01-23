/**
 * @file command.c
 * @brief Source file for serial command gimbal control module.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <ctype.h>

#include "pico/stdlib.h"

#include "command.h"
#include "pid.h"
#include "as5600.h"
#include "gimbal_configuration.h"

// Special character values
#define LF           10
#define CR           13
#define NO_VAL       254
#define ENDSTDIN     255

// Input radix
#define DECIMAL_BASE 10

// Input buffer and pointer
static char strg[100];
static int lp = 0;

/* Reset command buffer memory and pointer (to start) */
void resetSerialCommandInput() {
    memset(strg, 0, sizeof(strg));
    lp = 0;
}

/* 
    Read a single character from serial input into a buffer
    and return full command string when CR, LF or CRLF is encountered 
*/
char *readSerialCommand_nonBlocking() {
    char chr = getchar_timeout_us(0);

    // Skip if nothing to read (invalid) or error occurred
    if (chr == NO_VAL || chr == ENDSTDIN) {
        return NULL;
    }

    // Only process valid ASCII characters, CR, or LF
    if (chr >= 32 && chr <= 126) {
        strg[lp++] = chr;
    }

    bool endOfInput = (chr == CR && (getchar_timeout_us(0) == LF)) || (chr == CR) || (chr == LF);

    bool bufferFull = (lp == (sizeof(strg) - 1));

    // Terminate string on CR, LF, or CRLF or buffer full
    if (endOfInput || bufferFull) {
        strg[lp] = 0;  // Null-terminate
        if (strlen(strg) > 0) {
            return strg; // WARNING: INPUT BUFFER AND POINTER MUST BE RESET AFTER A COMMAND IS RETURNED
        }
    }

    return NULL;
}

/* Process, split and call execute on multiple commands */
void processCommands(char *input, gimbal_t *gimbal, gimbal_configuration_t *config) {
    char *command = strtok(input, " ");

    // Until no comands left to process, execute the current function
    while (command != NULL) {
        executeCommand(command, gimbal, config);
        command = strtok(NULL, " ");
    }
}

/* Execute individual commands */
void executeCommand(char *command, gimbal_t *gimbal, gimbal_configuration_t *config) {
    if (command == NULL || strlen(command) == 0) {
        printf("Empty command encountered.\n");
        return;
    }

    char *equals = strchr(command, '=');
    char *name = command;
    char *valueStr = NULL;

    // For commands where a value is expected, terminate at the equals and separate the value
    if (equals) {
        *equals = '\0';
        valueStr = equals + 1;
    }

    // ####################################################################################
    //                               AVAILABLE COMMANDS
    // ####################################################################################
    
    command_t commands[] = {
        // General
        {"help", "Show available commands."},

        // Config
        {"config-read", "Display the current gimbal configuration."},
        {"config-save", "Save the current gimbal configuration."},

        // Gimbal
        {"gimbal-read", "Display the current state of the gimbal."},
        {"gimbal-free", "Set the gimbal to free mode."},
        {"gimbal-arm", "Arm the gimbal and set initial pan and tilt setpoints."},
        {"gimbal-serialno", "Set the gimbal serial number (limited to uint8 values)."},

        // Stream
        {"stream-rate", "Set the global stream rate."},
        {"stream-global", "Enable or disable streaming (global)."},
        {"stream-pan-pos", "Enable or disable pan position streaming."},
        {"stream-pan-pid", "Enable or disable pan PID streaming."},
        {"stream-pan-motor", "Enable or disable pan motor streaming."},
        // {"stream-tilt-pos", "Enable or disable tilt position streaming."},
        // {"stream-tilt-pid", "Enable or disable tilt PID streaming."},
        // {"stream-tilt-motor", "Enable or disable tilt motor streaming."},

        // Pan
        {"pan-setpos", "Set the pan setpoint."},
        {"pan-pid-kp", "Set the proportional gain (Kp) for the pan PID controller."},
        {"pan-pid-ki", "Set the integral gain (Ki) for the pan PID controller."},
        {"pan-pid-kd", "Set the derivative gain (Kd) for the pan PID controller."},
        {"pan-pid-tau", "Set the derivative filter coefficient (tau) for the pan PID controller."},
        {"pan-pid-outLimMin", "Set the lower output limit for the pan PID controller."},
        {"pan-pid-outLimMax", "Set the upper output limit for the pan PID controller."},
        {"pan-pid-intLimMin", "Set the lower integrator limit for the pan PID controller."},
        {"pan-pid-intLimMax", "Set the upper integrator limit for the pan PID controller."},
        {"pan-pid-revert", "Revert to the last saved pan PID configuration."},

        // // Tilt
        // {"tilt-setpos", "Set the tilt setpoint."},
        // {"tilt-pid-kp", "Set the proportional gain (Kp) for the tilt PID controller."},
        // {"tilt-pid-ki", "Set the integral gain (Ki) for the tilt PID controller."},
        // {"tilt-pid-kd", "Set the derivative gain (Kd) for the tilt PID controller."},
        // {"tilt-pid-tau", "Set the derivative filter coefficient (tau) for the tilt PID controller."},
        // {"tilt-pid-outLimMin", "Set the lower output limit for the tilt PID controller."},
        // {"tilt-pid-outLimMax", "Set the upper output limit for the tilt PID controller."},
        // {"tilt-pid-intLimMin", "Set the lower integrator limit for the tilt PID controller."},
        // {"tilt-pid-intLimMax", "Set the upper integrator limit for the tilt PID controller."},
        // {"tilt-pid-revert", "Revert to the last saved tilt PID configuration."},
    };

    size_t numCommands = sizeof(commands) / sizeof(command_t);

    if (strcmp(name, "help") == 0) { // "h" shortcut
        printf("\n");

        // Command name and type section formatting
        const int nameWidth = 20;
        char currentSection[256] = "";

        printf("Available commands:\n");

        for (size_t i = 0; i < numCommands; ++i) {
            char section[256];
            const char *dashPosition = strchr(commands[i].name, '-');

            if (dashPosition != NULL) {
                size_t sectionLength = dashPosition - commands[i].name;
                strncpy(section, commands[i].name, sectionLength);
                section[sectionLength] = '\0';
            } else {
                // If no dash, use "General" as the section
                strcpy(section, "general");
            }

            // Print section name if starting to show commands from a new section
            if (strcmp(currentSection, section) != 0) {
                currentSection[0] = toupper(section[0]);
                strcpy(&currentSection[1], &section[1]);
                printf("  %s\n", currentSection);
                strcpy(currentSection, section);
            }

            printf("    %-*s %s\n", nameWidth, commands[i].name, commands[i].description);
        }

        printf("\n");
    }

    else if (strcmp(name, "config-read") == 0) { // "cr" shortcut
        printf("\n");
        displayGimbalConfiguration(config);
    } 
    
    else if (strcmp(name, "config-save") == 0) { // "cs" shortcut
        printf("\n");

        // Copy current gimbal gains to configuration before being saved
        config->panPositionController = gimbal->panPositionController;
        // config->tiltPositionController = gimbal->tiltPositionController;

        // Save configuration
        saveGimbalConfiguration(config);
        printf("\n");
        gimbal->savedConfiguration = true;
    } 
    
    else if (strcmp(name, "gimbal-read") == 0) { // "gr" shortcut
        printf("\n");
        displayGimbal(gimbal);
    } 

    else if (strcmp(name, "gimbal-free") == 0) { // "free" shortcut
        printf("\n");
        gimbal->gimbalMode = GIMBAL_MODE_FREE;
        printf("Gimbal disarmed\n");
        printf("\n");
    }

    else if (strcmp(name, "gimbal-arm") == 0) { // "arm" shortcut
        printf("\n");

        if (gimbal->panLowerLimit == GIMBAL_DEFAULT_SENTINEL || gimbal->panUpperLimit == GIMBAL_DEFAULT_SENTINEL) {
            printf("Pan axis limits are unset. Cannot arm.\n");
            printf("\n");
            return;
        }
        // if (gimbal->tiltLowerLimit == GIMBAL_DEFAULT_SENTINEL || gimbal->tiltUpperLimit == GIMBAL_DEFAULT_SENTINEL) {
        //     printf("Tilt axis limits are unset. Cannot arm.\n");
        //     printf("\n");
        //     return;
        // }

        // Get current gimbal angles and set setpoint for each axis to the current angle of that axis (smooth arming)
        uint16_t panRawAngle = AS5600_getRawAngle(&gimbal->panEncoder);
        // If not in the allowed range (between setpoints)
        gimbal->panPositionSetpoint = (float)panRawAngle;
        // uint16_t tiltRawAngle = AS5600_getRawAngle(&gimbal->tiltEncoder);
        // If not in the allowed range (between setpoints)
        // gimbal->tiltPositionSetpoint = (float)tiltRawAngle;

        gimbal->gimbalMode = GIMBAL_MODE_ARMED;
        printf("Gimbal armed\n");
        printf("\n");
    }

    else if (strcmp(name, "gimbal-serialno") == 0) { // "gsn" shortcut
        printf("\n");

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

        printf("\n");
    }

    else if (strcmp(name, "stream-rate") == 0) { // "sr" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || value < GIMBAL_FAST_STREAM_RATE || value > GIMBAL_SLOW_STREAM_RATE) {
                printf("Invalid value for 'stream-rate'. Use a value between %d-%d.\n",
                    GIMBAL_FAST_STREAM_RATE, GIMBAL_SLOW_STREAM_RATE);
            } else {
                gimbal->streamRate = (int)value;
                printf("Updated global stream rate to %d\n", (int)value);
            }
        } else {
            printf("Command 'stream-rate' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "stream-global") == 0) { // "sg" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
                printf("Invalid value for 'stream-global'. Use 0 (false) or 1 (true).\n");
            } else {
                gimbal->streaming = (bool)value;
                printf("%s streaming (global)\n", value ? "Enabled" : "Disabled");
            }
        } else {
            printf("Command 'stream-global' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "stream-pan-pos") == 0) { // "gsppos" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
                printf("Invalid value for 'stream-pan-pos'. Use 0 (false) or 1 (true).\n");
            } else {
                gimbal->panPositionStream = (bool)value;
                printf("%s pan position streaming\n", value ? "Enabled" : "Disabled");
            }
        } else {
            printf("Command 'stream-pan-pos' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "stream-pan-pid") == 0) { // "gsppid" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
                printf("Invalid value for 'stream-pan-pid'. Use 0 (false) or 1 (true).\n");
            } else {
                gimbal->panPositionStream = (bool)value;
                printf("%s pan pid streaming\n", value ? "Enabled" : "Disabled");
            }
        } else {
            printf("Command 'stream-pan-pid' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "stream-pan-motor") == 0) { // "gspm" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            long value = strtol(valueStr, &endptr, DECIMAL_BASE);

            if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
                printf("Invalid value for 'stream-pan-motor'. Use 0 (false) or 1 (true).\n");
            } else {
                gimbal->panPositionStream = (bool)value;
                printf("%s pan motor streaming\n", value ? "Enabled" : "Disabled");
            }
        } else {
            printf("Command 'stream-pan-motor' requires a value.\n");
        }

        printf("\n");
    }

    // else if (strcmp(name, "stream-tilt-pos") == 0) { // "gsppos" shortcut
    //     printf("\n");

    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         long value = strtol(valueStr, &endptr, DECIMAL_BASE);

    //         if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
    //             printf("Invalid value for 'stream-tilt-pos'. Use 0 (false) or 1 (true).\n");
    //         } else {
    //             gimbal->panPositionStream = (bool)value;
    //             printf("%s tilt position streaming\n", value ? "Enabled" : "Disabled");
    //         }
    //     } else {
    //         printf("Command 'stream-tilt-pos' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "stream-tilt-pid") == 0) { // "gsppid" shortcut
    //     printf("\n");

    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         long value = strtol(valueStr, &endptr, DECIMAL_BASE);

    //         if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
    //             printf("Invalid value for 'stream-tilt-pid'. Use 0 (false) or 1 (true).\n");
    //         } else {
    //             gimbal->panPositionStream = (bool)value;
    //             printf("%s tilt pid streaming\n", value ? "Enabled" : "Disabled");
    //         }
    //     } else {
    //         printf("Command 'stream-tilt-pid' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "stream-tilt-motor") == 0) { // "gspm" shortcut
    //     printf("\n");

    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         long value = strtol(valueStr, &endptr, DECIMAL_BASE);

    //         if (errno != 0 || *endptr != '\0' || (value != 0 && value != 1)) {
    //             printf("Invalid value for 'stream-tilt-motor'. Use 0 (false) or 1 (true).\n");
    //         } else {
    //             gimbal->panPositionStream = (bool)value;
    //             printf("%s tilt motor streaming\n", value ? "Enabled" : "Disabled");
    //         }
    //     } else {
    //         printf("Command 'stream-tilt-motor' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    else if (strcmp(name, "pan-setpos") == 0) { // "ps" shortcut
        printf("\n");

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

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-kp") == 0) { // "pkp" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value < 0) {
                printf("Invalid value for 'pan-pid-kp'. Gain values must be greater than or equal to 0.\n");
            } else {
                gimbal->panPositionController.Kp = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan Kp gain to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-kp' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-ki") == 0) { // "pki" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value < 0) {
                printf("Invalid value for 'pan-pid-ki'. Gain values must be greater than or equal to 0.\n");
            } else {
                gimbal->panPositionController.Ki = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan Ki gain to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-ki' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-kd") == 0) { // "pkd" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value < 0) {
                printf("Invalid value for 'pan-pid-kd'. Gain values must be greater than or equal to 0.\n");
            } else {
                gimbal->panPositionController.Kd = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan Kd gain to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-kd' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-tau") == 0) { // "ppt" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value < 0) {
                printf("Invalid value for 'pan-pid-tau'. Filter coefficients must be greater than or equal to 0.\n");
            } else {
                gimbal->panPositionController.tau = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan derivative low pass filter coefficient (tau) to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-tau' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-outLimMin") == 0) { // "polmin" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value > gimbal->panPositionController.outLimMax) {
                printf("Invalid value for 'pan-pid-outLimMin'. Lower limit must be lower than the upper limit.\n");
            } else {
                gimbal->panPositionController.outLimMin = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan pid ouput lower limit to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-outLimMin' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-outLimMax") == 0) { // "polmax" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value < gimbal->panPositionController.outLimMin) {
                printf("Invalid value for 'pan-pid-outLimMax'. Upper limit must be higher than the lower limit.\n");
            } else {
                gimbal->panPositionController.outLimMax = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan pid ouput upper limit to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-outLimMax' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-intLimMin") == 0) { // "pilmin" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value > gimbal->panPositionController.intLimMax) {
                printf("Invalid value for 'pan-pid-intLimMin'. Lower limit must be lower than the upper limit.\n");
            } else {
                gimbal->panPositionController.intLimMin = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan pid integrator lower limit to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-intLimMin' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-intLimMax") == 0) { // "pilmax" shortcut
        printf("\n");

        if (valueStr && valueStr[0] != '\0') {
            char *endptr;
            errno = 0;

            float value = strtof(valueStr, &endptr);

            if (errno != 0 || *endptr != '\0' || value < gimbal->panPositionController.intLimMin) {
                printf("Invalid value for 'pan-pid-intLimMax'. Upper limit must be higher than the lower limit.\n");
            } else {
                gimbal->panPositionController.intLimMax = value;
                gimbal->savedConfiguration = false;
                printf("Updated pan pid integrator upper limit to %f\n", value);
            }
        } else {
            printf("Command 'pan-pid-intLimMax' requires a value.\n");
        }

        printf("\n");
    }

    else if (strcmp(name, "pan-pid-revert") == 0) { // "ppr" shortcut
        printf("\n");
        gimbal->panPositionController = config->panPositionController;
        printf("Reverted back to last saved pan controller configuration\n");
        printf("\n");
    }

    // else if (strcmp(name, "tilt-setpos") == 0) { // "ts" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value < AS5600_RAW_ANGLE_MIN || value > AS5600_RAW_ANGLE_MAX) {
    //             printf("Invalid value for 'tilt-setpos'. Setpoint values must be between %u-%u.\n",
    //                 AS5600_RAW_ANGLE_MIN, AS5600_RAW_ANGLE_MAX);
    //         } else {
    //             gimbal->tiltPositionSetpoint = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt setpoint to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-setpos' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-kp") == 0) { // "tkp" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value < 0) {
    //             printf("Invalid value for 'tilt-pid-kp'. Gain values must be greater than or equal to 0.\n");
    //         } else {
    //             gimbal->tiltPositionController.Kp = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt Kp gain to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-kp' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-ki") == 0) { // "tki" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value < 0) {
    //             printf("Invalid value for 'tilt-pid-ki'. Gain values must be greater than or equal to 0.\n");
    //         } else {
    //             gimbal->tiltPositionController.Ki = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt Ki gain to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-ki' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-kd") == 0) { // "tkd" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value < 0) {
    //             printf("Invalid value for 'tilt-pid-kd'. Gain values must be greater than or equal to 0.\n");
    //         } else {
    //             gimbal->tiltPositionController.Kd = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt Kd gain to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-kd' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-tau") == 0) { // "tpt" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value < 0) {
    //             printf("Invalid value for 'tilt-pid-tau'. Filter coefficients must be greater than or equal to 0.\n");
    //         } else {
    //             gimbal->tiltPositionController.tau = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt derivative low pass filter coefficient (tau) to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-tau' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-outLimMin") == 0) { // "tolmin" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value > gimbal->tiltPositionController.outLimMax) {
    //             printf("Invalid value for 'tilt-pid-outLimMin'. Lower limit must be lower than the upper limit.\n");
    //         } else {
    //             gimbal->tiltPositionController.outLimMin = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt pid output lower limit to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-outLimMin' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-outLimMax") == 0) { // "tolmax" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value < gimbal->tiltPositionController.outLimMin) {
    //             printf("Invalid value for 'tilt-pid-outLimMax'. Upper limit must be higher than the lower limit.\n");
    //         } else {
    //             gimbal->tiltPositionController.outLimMax = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt pid output upper limit to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-outLimMax' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-intLimMin") == 0) { // "tilmin" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value > gimbal->tiltPositionController.intLimMax) {
    //             printf("Invalid value for 'tilt-pid-intLimMin'. Lower limit must be lower than the upper limit.\n");
    //         } else {
    //             gimbal->tiltPositionController.intLimMin = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt pid integrator lower limit to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-intLimMin' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-intLimMax") == 0) { // "tilmax" shortcut
    //     if (valueStr && valueStr[0] != '\0') {
    //         char *endptr;
    //         errno = 0;

    //         float value = strtof(valueStr, &endptr);

    //         if (errno != 0 || *endptr != '\0' || value < gimbal->tiltPositionController.intLimMin) {
    //             printf("Invalid value for 'tilt-pid-intLimMax'. Upper limit must be higher than the lower limit.\n");
    //         } else {
    //             gimbal->tiltPositionController.intLimMax = value;
    //             gimbal->savedConfiguration = false;
    //             printf("Updated tilt pid integrator upper limit to %f\n", value);
    //         }
    //     } else {
    //         printf("Command 'tilt-pid-intLimMax' requires a value.\n");
    //     }

    //     printf("\n");
    // }

    // else if (strcmp(name, "tilt-pid-revert") == 0) { // "tpr" shortcut
    //     gimbal->tiltPositionController = config->tiltPositionController;
    //     printf("Reverted back to last saved tilt controller configuration\n");
    //     printf("\n");
    // }
    
    else {
        printf("Unknown command: '%s'\n", name);
        printf("\n");
    }
}
