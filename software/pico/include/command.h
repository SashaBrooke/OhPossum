#ifndef COMMAND_H
#define COMMAND_H

#include "gimbal_configuration.h"

void resetSerialCommandInput();

char *readSerialCommand_nonBlocking();

void processCommands(char *input, gimbal_t *gimbal, gimbal_configuration_t *config);

void executeCommand(char *command, gimbal_t *gimbal, gimbal_configuration_t *config);

#endif