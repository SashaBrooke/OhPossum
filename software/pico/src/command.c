// TODO: Documentation
// Currently supports CR, LF and CRLF end of message
// Currently only supports a single input stream
// Currently limits the input buffer to a static length of 100

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"

#include "command.h"

#define LF          10
#define CR          13
#define NO_VAL      254
#define ENDSTDIN    255

static char strg[100];
static int lp = 0;

void initSerialCommandInput() {
    memset(strg, 0, sizeof(strg));
    lp = 0;
}

// Designed to be ran inside a repeating loop. Each loop iteration this
// function will instantly read a single character without holding up
// the execution of the program. Once the full message is read (CR, LF,
// CRLF encountered), it is returned.
char *readSerialCommand_nonBlocking() {
    char chr = getchar_timeout_us(0);

    // Skip if nothing to read (invalid) or error occurred
    if (chr == NO_VAL || chr == ENDSTDIN) {
        return NULL;
    }

    // Only process valid ASCII characters, CR, or LF
    if ((chr >= 32 && chr <= 126) || chr == CR || chr == LF) {
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
        memset(strg, 0, sizeof(strg));  // Clear the buffer
        lp = 0;  // Reset pointer
    }

    return NULL;
}

void *processCommands(char *input) {
    //
}
