#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "shell.h"
#include <stdint.h>
//#include "nodate.h"

int shell_cmd_power(shell_cmd_args *args);
int shell_cmd_balance(shell_cmd_args *args);
int shell_cmd_deviceinfo(shell_cmd_args *args);
int shell_cmd_uptime(shell_cmd_args *args);
int shell_cmd_imu(shell_cmd_args *args);

extern shell_cmds pencil_cmds;

/**
 * Definition of a single element
 */
typedef struct {
	/**
	 * Name of the element
	 */
	const char *name;

	/**
	 * Value of the elment
	 */
	uint8_t value;

} element_t;

/**
 * All elements of the list
 */
typedef struct {
	/**
	 * Number of elements
	 */
	unsigned char count;

	/**
	 * The elements
	 */
	element_t elements[];
} list_t;


#ifdef __cplusplus
}
#endif


#endif /* __COMMANDS_H__ */