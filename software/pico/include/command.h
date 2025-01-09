/**
 * @file command.h
 * @brief Defines the command interface for processing serial inputs and controlling the gimbal system.
 * 
 * This header provides functionalities for reading, processing, and executing serial commands. It defines
 * a set of available commands and their descriptions.
 */

#ifndef COMMAND_H
#define COMMAND_H

#include "gimbal_configuration.h"

/**
 * @struct command_t
 * @brief A structure containing information about a command.
 * 
 * Each command has a name and a corresponding description to describe its functionality.
 */
typedef struct {
    const char *name;
    const char *description;
} command_t;

/**
 * @brief Resets the serial command input buffer and pointer.
 * 
 * Clears the internal input buffer and resets the input pointer to allow for a fresh
 * command read. 
 * 
 * @warning This function must be called after processing a command with `readSerialCommand()`
 *          to ensure the next input is read correctly.
 */
void resetSerialCommandInput();

/**
 * @brief Reads a serial command without blocking.
 * 
 * This function is designed to be run in a repeating loop. Each loop iteration, a single
 * incoming character is read from the serial input without blocking the execution of the program.
 * Once a full message is received (ending with CR, LF, or CRLF), the full command is returned as a
 * null-terminated string. If no complete message is received, it returns `NULL`.
 * 
 * @note The input buffer is currently limited to 100 characters.
 * 
 * @warning After processing (returning) a command, the input buffer MUST be reset 
 *          using `resetSerialCommandInput()`.
 * 
 * @return A null-terminated string containing the full received command, or `NULL` if no command is ready.
 */
char *readSerialCommand_nonBlocking();

/**
 * @brief Processes a received serial command.
 * 
 * Splits the input command string into multiple individual commands (if multiple are present)
 * and executes each of them sequentially. Commands are expected to be space-separated. 
 * For example, a command string "<command1> <command2>" will
 * execute the <command1> and then <command2> commands sequentially.
 * 
 * @note See `executeCommands()` for more details of individual command format.
 * 
 * @param input The input command string to process.
 * @param gimbal A pointer to the `gimbal_t` structure representing the gimbal state.
 * @param config A pointer to the `gimbal_configuration_t` structure representing the gimbal configuration.
 */
void processCommands(char *input, gimbal_t *gimbal, gimbal_configuration_t *config);

/**
 * @brief Executes a single gimbal control command.
 * 
 * Parses and executes the provided command string (a single command). 
 * The command may contain a name and an optional value separated 
 * by `=` (e.g., "<command1>=<value1>"). Commands are validated, and any invalid input
 * will result in an error message.
 * 
 * @note For information on supported commands, see `commands`.
 * 
 * @param command The command string to execute.
 * @param gimbal A pointer to the `gimbal_t` structure representing the gimbal state.
 * @param config A pointer to the `gimbal_configuration_t` structure representing the gimbal configuration.
 */
void executeCommand(char *command, gimbal_t *gimbal, gimbal_configuration_t *config);

#endif // COMMAND_H
