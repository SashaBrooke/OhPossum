// TODO: Documentation
// Currently supports CR, LF and CRLF end of message
// Currently only supports a single input stream
// Currently limits the input buffer to a static length of 100

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"

#include "command.h"
#include "pid.h"
#include "as5600.h"
#include "gimbal_configuration.h"

#define LF          10
#define CR          13
#define NO_VAL      254
#define ENDSTDIN    255

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

    const char *supportedCommands[] = {"gimbal-mode"};
    size_t supportedCommandsCount = sizeof(supportedCommands) / sizeof(supportedCommands[0]);

    char *equals = strchr(command, '=');
    char *name = command;
    char *valueStr = NULL;

    if (equals) {
        *equals = '\0';
        valueStr = equals + 1;
    }

    bool found = false;
    for (size_t i = 0; i < supportedCommandsCount; ++i) {
        if (strcmp(name, supportedCommands[i]) == 0) {
            found = true;

            if (strcmp(name, "gimbal-mode") == 0) {
                if (valueStr) {
                    int value = atoi(valueStr);
                    if (value > GIMBAL_MODE_LOWER_LIMIT && value < GIMBAL_MODE_UPPER_LIMIT) {
                        gimbal->gimbalMode = value;
                        printf("Updated gimbalMode to %d\n", gimbal->gimbalMode);
                    } else {
                        printf("Gimbal mode values must be between %d-%d\n", 
                               GIMBAL_MODE_LOWER_LIMIT + 1, GIMBAL_MODE_UPPER_LIMIT - 1);
                    }
                } else {
                    printf("Command 'gimbal-mode' requires a value.\n");
                }
            }
            break;
        }
    }

    if (!found) {
        printf("Unknown command: %s\n", name);
    }
}
