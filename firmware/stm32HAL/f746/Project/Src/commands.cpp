//#include "tick.h"
#include <commands.hpp>
#include "terminal.h"
#include "hardware.hpp"
#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdio.h>

shell_cmds pencil_cmds =
{
	.count = 5,
	.cmds  = {
		{
			.cmd     = "charge",
			.desc    = "Control motor power",
			.func    = shell_cmd_power,
		},
		{
			.cmd     = "imu",
			.desc    = "IMU reading",
			.func    = shell_cmd_imu,
		},
		{
			.cmd     = "balance",
			.desc    = "Balance with left, right or both motors",
			.func    = shell_cmd_balance,
		},
		{
			.cmd     = "deviceinfo",
			.desc    = "Show device information",
			.func    = shell_cmd_deviceinfo,
		},
		{
			.cmd     = "uptime",
			.desc    = "Tell how long the system has been running.",
			.func    = shell_cmd_uptime,
		},
	},
};

list_t power_options =
{
	.count    = 4,
	.elements = {
		{
			.name     = "none",
			.value    = 0,
		},
		{
			.name     = "left",
			.value    = 1,
		},
		{
			.name     = "right",
			.value    = 2,
		},
		{
			.name     = "both",
			.value    = 3,
		},
	},
};

list_t balance_options =
{
	.count    = 4,
	.elements = {
		{
			.name     = "none",
			.value    = 0,
		},
		{
			.name     = "left",
			.value    = 0,
		},
		{
			.name     = "right",
			.value    = 0,
		},
		{
			.name     = "both",
			.value    = 0,
		},
	},
};

/*
 * @brief report imu reading
 */
int shell_cmd_imu(shell_cmd_args *args)
{
	bool update_available = imu.update_IMU_values();
	if (!update_available) {
		printf("no update available \n");
		return 0;
	}
	float theta, integral, speed, zero, omega;
	imu.get_theta_values(theta, integral, speed, zero);
	imu.get_omega_values(omega,  integral,  speed, zero);
	printf("omega = %f, theta = %f \n", omega, theta);
	return 0;
}

/*
 * @brief processes the power command
 */
int shell_cmd_power(shell_cmd_args *args)
{
	int voltage = 0;
	if (args->count != 2) {
		printf("Invalid power option arguments\n");
		return 0;
	}
	else {
		voltage = atoi(args->args[1].val);
	}
	for (int i = 0; i < power_options.count; i++)
	{
		if (strcmp(args->args[0].val, power_options.elements[i].name) == 0)
		{
			// deactivate the controllers before setting fixed voltages for the motors
			left_controller_active = false;
			right_controller_active = false;
			switch (power_options.elements[i].value) {
			case 0:
				leftMotor.set_voltage(0);
				rightMotor.set_voltage(0);
				break;
			case 1:
				leftMotor.set_voltage(voltage);
				break;
			case 2:
				rightMotor.set_voltage(voltage);
				break;
			case 3:
				leftMotor.set_voltage(voltage);
				rightMotor.set_voltage(voltage);
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
 * @brief process the balance command
 */
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

/*
 * @brief processes the deviceinfo menu option
 */
int shell_cmd_deviceinfo(shell_cmd_args *args)
{
	DisplaySystemInformation();
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

