#ifndef COMMAND_INPUT_H
#define COMMAND_INPUT_H

void initSerialCommandInput();

char *readSerialCommand_nonBlocking();

#endif