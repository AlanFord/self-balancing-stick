//#include "tick.h"
#include "commands.h"
#include "terminal.h"
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
	return 0;
}

/*
 * @brief processes the power command
 */
int shell_cmd_power(shell_cmd_args *args)
{
	for (int i = 0; i < power_options.count; i++)
	{
		if (strcmp(args->args[0].val, power_options.elements[i].name) == 0)
		{
			for (int i = 0; i < power_options.count; i++) {
				power_options.elements[i].value = 0;
			}
			power_options.elements[i].value = 1;
			printf("Power option was %s \n",args->args[0].val);
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

