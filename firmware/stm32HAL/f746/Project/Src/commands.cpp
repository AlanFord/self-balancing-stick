//#include "tick.h"
#include <commands.hpp>
#include "terminal.h"
#include "hardware.hpp"
#include "appEntry.hpp"
#include "stm32f7xx_hal.h"
#include <string.h>
//#include <string>
#include <cstdio>
shell_cmds pencil_cmds =
{
	.count = 29,
	.cmds  = {
		{
			.cmd     = "y",
			.desc    = "Charge left motor",
			.func    = shell_cmd_charge_left,
		},
		{
			.cmd     = "h",
			.desc    = "Charge right motor",
			.func    = shell_cmd_charge_right,
		},
		{
			.cmd     = "n",
			.desc    = "Charge both motors",
			.func    = shell_cmd_charge_both,
		},
		{
			.cmd     = "=",
			.desc    = "Shutdown both motors",
			.func    = shell_cmd_shutdown_both,
		},
		{
			.cmd     = "u",
			.desc    = "Balance with left motor",
			.func    = shell_cmd_balance_left,
		},
		{
			.cmd     = "j",
			.desc    = "Balance with right motor",
			.func    = shell_cmd_balance_right,
		},
		{
			.cmd     = "m",
			.desc    = "Balance with both motors",
			.func    = shell_cmd_balance_both,
		},
		{
			.cmd     = "z",
			.desc    = "Change zeros",
			.func    = shell_cmd_change_zeros,
		},
		{
			.cmd     = "q",
			.desc    = "Set theta Kp",
			.func    = shell_cmd_set_theta_kp,
		},
		{
			.cmd     = "w",
			.desc    = "Set theta Ki",
			.func    = shell_cmd_set_theta_ki,
		},
		{
			.cmd     = "e",
			.desc    = "Set theta Kd",
			.func    = shell_cmd_set_theta_kd,
		},
		{
			.cmd     = "r",
			.desc    = "Set theta Ks",
			.func    = shell_cmd_set_theta_ks,
		},
		{
			.cmd     = "o",
			.desc    = "Set theta Kt",
			.func    = shell_cmd_set_theta_kt,
		},
		{
			.cmd     = "a",
			.desc    = "Set omega Kp",
			.func    = shell_cmd_set_omega_kp,
		},
		{
			.cmd     = "s",
			.desc    = "Set omega Ki",
			.func    = shell_cmd_set_omega_ki,
		},
		{
			.cmd     = "d",
			.desc    = "Set omega Kd",
			.func    = shell_cmd_set_omega_kd,
		},
		{
			.cmd     = "f",
			.desc    = "Set omega Ks",
			.func    = shell_cmd_set_omega_ks,
		},
		{
			.cmd     = "l",
			.desc    = "Set omega Kt",
			.func    = shell_cmd_set_omega_kt,
		},
		{
			.cmd     = "t",
			.desc    = "Set left Target Voltage",
			.func    = shell_cmd_set_left_target_voltage,
		},
		{
			.cmd     = "g",
			.desc    = "Set right Target Voltage",
			.func    = shell_cmd_set_right_target_voltage,
		},
		{
			.cmd     = "1",
			.desc    = "Set the angle average filter",
			.func    = shell_cmd_set_angle_average_filter,
		},
		{
			.cmd     = "2",
			.desc    = "Set the theta zero filter",
			.func    = shell_cmd_set_theta_zero_filter,
		},
		{
			.cmd     = "3",
			.desc    = "Set the omega zero filter",
			.func    = shell_cmd_set_omega_zero_filter,
		},
		{
			.cmd     = "4",
			.desc    = "Set the angle smoothed filter",
			.func    = shell_cmd_set_angle_smoothed_filter,
		},
		{
			.cmd     = "9",
			.desc    = "Set the friction value",
			.func    = shell_cmd_set_friction_value,
		},
		{
			.cmd     = "p",
			.desc    = "Show extended data",
			.func    = shell_cmd_show_extended_data,
		},
		{
			.cmd     = "deviceinfo",
			.desc    = "Show device information",
			.func    = shell_cmd_deviceinfo,
		},
		{
			.cmd     = "help",
			.desc    = "Show commands",
			.func    = shell_cmd_show_help,
		},
		{
			.cmd     = "uptime",
			.desc    = "Show uptime",
			.func    = shell_cmd_uptime,
		},
	},
};

list_t zero_options =
{
	.count    = 5,
	.elements = {
		{
			.name     = "z",
			.value    = 0,
		},
		{
			.name     = "x",
			.value    = 1,
		},
		{
			.name     = "c",
			.value    = 2,
		},
		{
			.name     = "o",
			.value    = 3,
		},
		{
			.name     = "l",
			.value    = 4,
		},
	},
};

/*
 * @brief power up the left motor
 */
int shell_cmd_charge_left(shell_cmd_args *args)
{
	leftController_ptr->set_mode(MANUAL);
	rightController_ptr->set_mode(OFF);
	return 0;
}

/*
 * @brief power up the right motor
 */
int shell_cmd_charge_right(shell_cmd_args *args)
{
	leftController_ptr->set_mode(OFF);
	rightController_ptr->set_mode(MANUAL);
	return 0;
}

/*
 * @brief power up both motors
 */
int shell_cmd_charge_both(shell_cmd_args *args)
{
	leftController_ptr->set_mode(MANUAL);
	rightController_ptr->set_mode(MANUAL);
	return 0;
}
/*
 * @brief shutdown both motors
 */
int shell_cmd_shutdown_both(shell_cmd_args *args)
{
	leftController_ptr->set_mode(OFF);
	rightController_ptr->set_mode(OFF);
	// as insurance, kill the motors directly
	leftMotor_ptr->set_voltage(0);
	rightMotor_ptr->set_voltage(0);
	return 0;
}

/*
 * @brief balance using left motor
 */
int shell_cmd_balance_left(shell_cmd_args *args)
{
	leftController_ptr->set_mode(AUTO);
	rightController_ptr->set_mode(OFF);
	return 0;
}

/*
 * @brief balance using right motor
 */
int shell_cmd_balance_right(shell_cmd_args *args)
{
	leftController_ptr->set_mode(OFF);
	rightController_ptr->set_mode(AUTO);
	return 0;
}

/*
 * @brief balance using both motors
 */
int shell_cmd_balance_both(shell_cmd_args *args)
{
	leftController_ptr->set_mode(AUTO);
	rightController_ptr->set_mode(AUTO);
	return 0;
}

/*
 * @brief processes the zero command
 */
int shell_cmd_change_zeros(shell_cmd_args *args)
{
	// FIXME: sort out the secondary options
	int voltage = 0;
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	else {
		voltage = atoi(args->args[1].val);
	}
	for (int i = 0; i < zero_options.count; i++)
	{
		if (strcmp(args->args[0].val, zero_options.elements[i].name) == 0)
		{
			// deactivate the controllers before setting fixed voltages for the motors
			left_controller_active = false;
			right_controller_active = false;
			switch (zero_options.elements[i].value) {
			case 0:
				leftMotor_ptr->set_voltage(0);
				rightMotor_ptr->set_voltage(0);
				break;
			case 1:
				leftMotor_ptr->set_voltage(voltage);
				break;
			case 2:
				rightMotor_ptr->set_voltage(voltage);
				break;
			case 3:
				leftMotor_ptr->set_voltage(voltage);
				rightMotor_ptr->set_voltage(voltage);
				break;
			}
			printf("voltage set to %i \n", voltage);
			return 0;
		}
	}
	printf("Invalid power option \n");
	return 0;
}

/*
 * @brief set the theta Kp value
 */
int shell_cmd_set_theta_kp(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	leftController_ptr->set_Kp(newVal);
	return 0;
}

/*
 * @brief set the theta Ki value
 */
int shell_cmd_set_theta_ki(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	leftController_ptr->set_Ki(newVal);
	return 0;
}

/*
 * @brief set the theta Kd value
 */
int shell_cmd_set_theta_kd(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	leftController_ptr->set_Kd(newVal);
	return 0;
}

/*
 * @brief set the theta Ks value
 */
int shell_cmd_set_theta_ks(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	leftController_ptr->set_Ks(newVal);
	return 0;
}

/*
 * @brief set the theta Kt value
 */
int shell_cmd_set_theta_kt(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	leftController_ptr->set_Kt(newVal);
	return 0;
}

/*
 * @brief set the omega Kp value
 */
int shell_cmd_set_omega_kp(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_Kp(newVal);
	return 0;
}

/*
 * @brief set the omega Ki value
 */
int shell_cmd_set_omega_ki(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_Kp(newVal);
	return 0;
}

/*
 * @brief set the omega Kd value
 */
int shell_cmd_set_omega_kd(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_Kp(newVal);
	return 0;
}

/*
 * @brief set the omega Ks value
 */
int shell_cmd_set_omega_ks(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_Kp(newVal);
	return 0;
}

/*
 * @brief set the omega Kt value
 */
int shell_cmd_set_omega_kt(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_Kt(newVal);
	return 0;
}

/*
 * @brief set left target voltage
 */
int shell_cmd_set_left_target_voltage(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	int newVal = strtol(args->args[1].val,NULL,10);
	leftController_ptr->set_default_voltage(newVal);
	return 0;
}

/*
 * @brief set right target voltage
 */
int shell_cmd_set_right_target_voltage(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	int newVal = strtol(args->args[1].val,NULL,10);
	rightController_ptr->set_default_voltage(newVal);
	return 0;
}

/*
 * @brief set angle average filter
 */
int shell_cmd_set_angle_average_filter(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	int newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_angle_Average_Filter(newVal);
	leftController_ptr->set_angle_Average_Filter(newVal);
	return 0;
}

/*
 * @brief set theta zero filter
 */
int shell_cmd_set_theta_zero_filter(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	leftController_ptr->set_Zero(newVal);
	return 0;
}

/*
 * @brief set omega zero filter
 */
int shell_cmd_set_omega_zero_filter(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_Zero(newVal);
	return 0;
}

/*
 * @brief set angle smoothed filter
 */
int shell_cmd_set_angle_smoothed_filter(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	int newVal = strtof(args->args[1].val,NULL);
	rightController_ptr->set_angle_Smoothed_Filter(newVal);
	leftController_ptr->set_angle_Smoothed_Filter(newVal);
	return 0;
}

/*
 * @brief set friction value
 */
int shell_cmd_set_friction_value(shell_cmd_args *args)
{
	if (args->count != 2) {
		printf("Invalid zero option arguments\n");
		return 0;
	}
	float newVal = strtof(args->args[1].val,NULL);
	leftController_ptr->set_friction(newVal);
	rightController_ptr->set_friction(newVal);
	return 0;
}

const char* statusString = R"(

            Kp	Ki	Kd	Ks	Kt	Ktd
theta PID:	%f	%f	%f	%f	%f	%f
omega PID:	%f	%f	%f	%f	%f	%f	

Angle Average Filter:	%f.3
Theta Zero Filter:	%f.3
Omega Zero Filter:	%f.3

left Friction:	%i
right Friction:	%i)";

/*
 * @brief show extended data
 */
int shell_cmd_show_extended_data(shell_cmd_args *args)
{
	// FIXME turn off motors
	printf(statusString,
			leftController_ptr->get_Kp(),
			leftController_ptr->get_Ki(),
			leftController_ptr->get_Kd(),
			leftController_ptr->get_Ks(),
			leftController_ptr->get_Kt(),
			leftController_ptr->get_Ktd(),
			rightController_ptr->get_Kp(),
			rightController_ptr->get_Ki(),
			rightController_ptr->get_Kd(),
			rightController_ptr->get_Ks(),
			rightController_ptr->get_Kt(),
			rightController_ptr->get_Ktd(),
			leftController_ptr->get_angle_Average_Filter(),
			leftController_ptr->get_Zero_Filter(),
			rightController_ptr->get_Zero_Filter(),
			leftController_ptr->get_friction(),
			rightController_ptr->get_friction()
			);
	return 0;
}

/*
 * @brief process the balance command
 */
/*
int shell_cmd_balance(shell_cmd_args *args)
{
	for (int i = 0; i < balance_options.count; i++)
	{
		if (strcmp(args->args[0].val, balance_options.elements[i].name) == 0)
		{
			for (int i = 0; i < balance_options.count; i++) {
				balance_options.elements[i].value = 0;
			}
			balance_options.elements[i].value = 1;
			printf("Balance option was %s \n",args->args[0].val);
			return 0;
		}
	}
	printf("Invalid balance option \n");
	return 0;
}
*/

/*
 * @brief processes the deviceinfo menu option
 */
int shell_cmd_deviceinfo(shell_cmd_args *args)
{
	DisplaySystemInformation();
	return 0;
}

/*
 * @brief show help
 */
int shell_cmd_show_help(shell_cmd_args *args)
{
	// FIXME add secondary info about setting zeros
	for (int i = 0; i< pencil_cmds.count; i++){
		printf("commands:\n%s:\t%s\n",pencil_cmds.cmds[i].cmd, pencil_cmds.cmds[i].desc);
	}
	printf("\n");
	return 0;
}

/*
 * @brief processes the uptime menu option
 */
int shell_cmd_uptime(shell_cmd_args *args)
{
	uint32_t uptime = HAL_GetTick();
	int seconds = ((int)(uptime)) % 60;
	int minutes = ((int)(uptime / (60))) % 60;
	int hours   = ((int)(uptime / (60 * 60))) % 24;
	int days    = ((int)(uptime / (60 * 60 * 24))) % 7;
	//int weeks   = ((int)(uptime / (60*60*24*7))) % 52;
	int years   = ((int)(uptime / (60 * 60 * 24 * 365))) % 10;
	//int decades = ((int)(uptime / (60*60*24*365*10)));
	printf("System Up Time: %i Years, %i Days, %i Hours, %i Minutes, %i Seconds\r\n",
			   years, days, hours, minutes, seconds);
	return 0;
}

