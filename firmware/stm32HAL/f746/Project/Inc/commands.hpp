#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "shell.h"
#include <stdint.h>
//#include "nodate.h"

int shell_cmd_charge_left(shell_cmd_args *args);
int shell_cmd_charge_right(shell_cmd_args *args);
int shell_cmd_charge_both(shell_cmd_args *args);
int shell_cmd_shutdown_both(shell_cmd_args *args);
int shell_cmd_balance_left(shell_cmd_args *args);
int shell_cmd_balance_right(shell_cmd_args *args);
int shell_cmd_balance_both(shell_cmd_args *args);
int shell_cmd_change_zeros(shell_cmd_args *args);
int shell_cmd_set_theta_kp(shell_cmd_args *args);
int shell_cmd_set_theta_ki(shell_cmd_args *args);
int shell_cmd_set_theta_kd(shell_cmd_args *args);
int shell_cmd_set_theta_ks(shell_cmd_args *args);
int shell_cmd_set_theta_kt(shell_cmd_args *args);
int shell_cmd_set_omega_kp(shell_cmd_args *args);
int shell_cmd_set_omega_ki(shell_cmd_args *args);
int shell_cmd_set_omega_kd(shell_cmd_args *args);
int shell_cmd_set_omega_ks(shell_cmd_args *args);
int shell_cmd_set_omega_kt(shell_cmd_args *args);
int shell_cmd_set_left_target_voltage(shell_cmd_args *args);
int shell_cmd_set_right_target_voltage(shell_cmd_args *args);
int shell_cmd_set_angle_average_filter(shell_cmd_args *args);
int shell_cmd_set_theta_zero_filter(shell_cmd_args *args);
int shell_cmd_set_omega_zero_filter(shell_cmd_args *args);
int shell_cmd_set_angle_smoothed_filter(shell_cmd_args *args);
int shell_cmd_set_friction_value(shell_cmd_args *args);
int shell_cmd_show_extended_data(shell_cmd_args *args);
//int shell_cmd_balance(shell_cmd_args *args);
int shell_cmd_deviceinfo(shell_cmd_args *args);
int shell_cmd_show_help(shell_cmd_args *args);
int shell_cmd_uptime(shell_cmd_args *args);

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
	 * Value of the elmeent
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
