// TODO: Documentation
// Currently supports CR, LF and CRLF end of message

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"

#define LF          10
#define CR          13
#define NO_VAL      254
#define ENDSTDIN    255

int main() {
    stdio_init_all();

    char strg[100];
    char chr;
    int lp = 0;

    memset(strg, 0, sizeof(strg));

    while (true) {
        chr = getchar_timeout_us(0);

        // Skip if nothing to read (invalid) or error occurred
        if (chr == NO_VAL || chr == ENDSTDIN) {
            continue;
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
                printf("You wrote - %s\n", strg);
            }
            memset(strg, 0, sizeof(strg));  // Clear the buffer
            lp = 0;  // Reset pointer
        }
    }

    return 0;
}
