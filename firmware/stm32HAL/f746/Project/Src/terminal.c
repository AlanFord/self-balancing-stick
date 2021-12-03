///////////////////////////////////////////////////////////////////////////////
/// \file Terminal.c
///
///	\author Alan Ford
///////////////////////////////////////////////////////////////////////////////
#include "common.h"
#include "serial.h"
#include "shell.h"
#include "commands.h"
#include <stdio.h>
#include "terminal.h"

///////////////////////////////////////////////////////////////////////////////
/// \brief Defines our terminal buffer size which in turn set the longest command
///////////////////////////////////////////////////////////////////////////////
#define TERMINAL_BUFFER_SIZE 100

///////////////////////////////////////////////////////////////////////////////
/// \brief signaling LED
///////////////////////////////////////////////////////////////////////////////
// TODO: should this somehow be in the "mcu" folder?
//static LED signal_led(GPIO_PORT_B, 0);

///////////////////////////////////////////////////////////////////////////////
/// \brief The terminal data buffer
///////////////////////////////////////////////////////////////////////////////
static char Buffer[TERMINAL_BUFFER_SIZE];

///////////////////////////////////////////////////////////////////////////////
/// \brief this message has the system information data that we want to display
/// on the terminal
///////////////////////////////////////////////////////////////////////////////
static const char SystemMessageString[] =
		"-----------------------------------\r\n"
				"Firm : " FIRMWARE_VERSION "\r\n"
		"Hard : " HARDWARE_VERSION "\r\n"
		COMPILED_DATA_TIME "\r\n"
		"-----------------------------------\r\n";

///////////////////////////////////////////////////////////////////////////////
/// \brief Send the system information to the computer. This first clear the
///	computer terminal screen.
///////////////////////////////////////////////////////////////////////////////
void DisplaySystemInformation(void) {
	SerialPort.SendByte(0x0C);  // clear terminal
	SerialPort.SendString(&SystemMessageString[0]);
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Init the terminal program
///////////////////////////////////////////////////////////////////////////////
void Terminal_Init(void) {
	//Tick::Init();
	SerialPort.Open();
	setbuf(stdout, NULL); //prepare for future "unbuffered" use of printf

	DisplaySystemInformation();
}

int serve_command_prompt(char *buffer, int bufferLength, const char *prompt) {
	uint8_t SerialTempData = 0; // hold the new byte from the serial fifo

	static char initialized = FALSE;
	static char *p;

	if (!initialized) {
		p = buffer;
		*p = '\0';
		printf("%s", prompt);
		initialized = TRUE;
	}

	while (TRUE == SerialPort.GetByte(&SerialTempData)) {
		char c = SerialTempData;
		switch (c) {
		// https://en.wikipedia.org/wiki/ASCII#ASCII_control_characters
		case '\r':
			break;  // assume \r will be followed by \n, so wait for that

		case '\n':
			printf("\r\n");
			initialized = FALSE;
			return shell_str_len(buffer);

		case '\b':
			if (p > buffer)
				*--p = '\0';
			printf("\b \b");
			break;

		case 0x15: // CTRL-U
			while (p != buffer) {
				printf("\b \b");
				--p;
			}
			*p = '\0';
			break;

		case '\e': // ESC
			printf("^[\r\n");
			initialized = FALSE;
			break;

		case 0x7f: // 127, or delete
			if (p > buffer)
				*--p = '\0';
			printf("%c", c);
			break;

		default:
			if (p < buffer + bufferLength - 1 && c >= ' ' && c < 127) {
				*p++ = c;
				*p = '\0';
				printf("%c", c);
			} else
				// if the buffer would overflow, simply echo back to the sender
				printf("%c", c);
			break;
		}
	}

	return 0;
}

int shell_process(char *cmd_line) {
	int ret = shell_process_cmds(&pencil_cmds, cmd_line);

	switch (ret) {
	case SHELL_PROCESS_ERR_CMD_UNKN:
		// Unknown command
		printf("ERROR: Unknown command: '%s'\r\n", cmd_line);
		break;
	case SHELL_PROCESS_ERR_ARGS_LEN:
		// Argument to long
		printf("ERROR: Argument to long: '%s'\r\n", cmd_line);
		break;
	case SHELL_PROCESS_ERR_ARGS_MAX:
		// Too many arguments
		printf("ERROR: Too many arguments: '%s'\r\n", cmd_line);
		break;
	default:
		// OK
		break;
	}
	return ret;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief process the buffer data and extract the commands
///
///	\return TRUE success. FALSE no error
///
///	\todo update the document return state value. need to implement
///////////////////////////////////////////////////////////////////////////////
int_fast8_t Terminal_Process(void) {
	if (serve_command_prompt(Buffer, TERMINAL_BUFFER_SIZE, "pencil> ") > 0) {
		if (shell_process(Buffer) == SHELL_PROCESS_OK) {
			return TRUE;
		}
	}
	return FALSE;
}

