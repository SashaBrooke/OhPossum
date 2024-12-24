#ifndef COMMAND_H
#define COMMAND_H

void initSerialCommandInput();

char *readSerialCommand_nonBlocking();

#endif