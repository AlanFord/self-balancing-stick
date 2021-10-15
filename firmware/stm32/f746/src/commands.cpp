#include "tick.h"
#include "commands.h"
#include <string.h>

shell_cmds pencil_cmds =
{
	.count = 8,
	.cmds  = {
		{
			.cmd     = "list",
			.desc    = "List available inputs/outputs/commands/options/flags/connections",
			.func    = shell_cmd_list,
		},
		{
			.cmd     = "deviceinfo",
			.desc    = "Show device information",
			.func    = shell_cmd_deviceinfo,
		},
		{
			.cmd     = "enable",
			.desc    = "Enable an output or an option",
			.func    = shell_cmd_enable,
		},
		{
			.cmd     = "disable",
			.desc    = "Disable an output or an option",
			.func    = shell_cmd_disable,
		},
		{
			.cmd     = "toggle",
			.desc    = "Toggle an output (LEDx)",
			.func    = shell_cmd_toggle,
		},
		{
			.cmd     = "get",
			.desc    = "Get LED0 or GET BUTTON1",
			.func    = shell_cmd_get,
		},
		{
			.cmd     = "ping",
			.desc    = "Send a 'pong' back",
			.func    = shell_cmd_ping,
		},
		{
			.cmd     = "uptime",
			.desc    = "Tell how long the system has been running.",
			.func    = shell_cmd_uptime,
		},
	},
};

int shell_cmd_list(shell_cmd_args *args)
{
	if (args->count == 1)
	{
		if (strcmp(args->args[0].val, "commands") == 0)
		{
			for (int i = 0; i < pencil_cmds.count; i++) // -2 -> don't list CONNECT, DISCONNECT
			{
				//cio_printf("%s --> %s\r\n", pencil_cmds.cmds[i].cmd, pencil_cmds.cmds[i].desc);
			}
		}
		else if (strcmp(args->args[0].val, "inputs") == 0)
		{
			//for (int i = 0; i < inputs.count; i++)
			//{
			//	//cio_printf("%s\r\n", inputs.elements[i].name);
			//}

		}
		else if (strcmp(args->args[0].val, "outputs") == 0)
		{
			//for (int i = 0; i < outputs.count; i++)
			//{
				//cio_printf("%s\r\n", outputs.elements[i].name);
			//}

		}
		else if (strcmp(args->args[0].val, "options") == 0)
		{
			//for (int i = 0; i < options.count; i++)
			//{
				//cio_printf("%s\r\n", options.elements[i].name);
			//}

		}
		else if (strcmp(args->args[0].val, "flags") == 0)
		{
			//for (int i = 0; i < flags.count; i++)
			//{
				//cio_printf("%s\r\n", flags.elements[i].name);
			//}

		}
		else
		{
			printf("[1] Unknown argument: '%s'\r\n", args->args[0].val);
		}
	}

	return 0;
}

int shell_cmd_deviceinfo(shell_cmd_args *args)
{
	printf("Device Type: F0-Discovery Demo\r\n"
			  "Hardware Revision: 1.0\r\n"
			  "Firmware Version: 1.8\r\n"
			  "Build date: " __DATE__ "\r\n"
			  "URL: www.jann.cc\r\n");
	return 0;
}

int shell_cmd_ping(shell_cmd_args *args)
{
	printf("pong\r\n");
	return 0;
}

int shell_cmd_get(shell_cmd_args *args)
{
	/*
	for (int i = 0; i < inputs.count; i++)
	{
		if (strcmp(args->args[0].val, inputs.elements[i].name) == 0)
		{
			//cio_printf("%i\r\n", STM_EVAL_PBGetState(inputs.elements[i].value));
			return 0;
		}
	}
	for (int i = 0; i < flags.count; i++)
	{
		if (strcmp(args->args[0].val, flags.elements[i].name) == 0)
		{
			//cio_printf("%i\r\n", flags.elements[i].value);
			return 0;
		}
	}
	for (int i = 0; i < options.count; i++)
	{
		if (strcmp(args->args[0].val, options.elements[i].name) == 0)
		{
			//printf("%i\r\n", options.elements[i].value);
			return 0;
		}
	}
	for (int i = 0; i < outputs.count; i++)
	{
		if (strcmp(args->args[0].val, outputs.elements[i].name) == 0)
		{
			//printf("%i\r\n", GPIO_ReadOutputDataBit(GPIO_PORT[outputs.elements[i].value], GPIO_PIN[outputs.elements[i].value]));
			return 0;
		}
	} */
	return 0;
}

int shell_cmd_enable(shell_cmd_args *args)
{ /*
	for (int i = 0; i < options.count; i++)
	{
		//if (strcmp(args->args[0].val, options.elements[i].name) == 0)
		//{
		//	options.elements[i].value = 1;
		//	return 0;
		//}
	}
	for (int i = 0; i < outputs.count; i++)
	{
		//if (strcmp(args->args[0].val, outputs.elements[i].name) == 0)
		//{
		//	//STM_EVAL_LEDOn(outputs.elements[i].value);
		//	return 0;
		//}
	} */
	return 0;
}

int shell_cmd_disable(shell_cmd_args *args)
{ /*
	for (int i = 0; i < options.count; i++)
	{
		if (strcmp(args->args[0].val, options.elements[i].name) == 0)
		{
			options.elements[i].value = 0;
			return 0;
		}
	}
	for (int i = 0; i < outputs.count; i++)
	{
		if (strcmp(args->args[0].val, outputs.elements[i].name) == 0)
		{
			//STM_EVAL_LEDOff(outputs.elements[i].value);
			return 0;
		}
	} */
	return 0;
}

int shell_cmd_toggle(shell_cmd_args *args)
{ /*
	for (int i = 0; i < outputs.count; i++)
	{
		if (strcmp(args->args[0].val, outputs.elements[i].name) == 0)
		{
			//STM_EVAL_LEDToggle(outputs.elements[i].value);
			return 0;
		}
	} */
	return 0;
}

int shell_cmd_uptime(shell_cmd_args *args)
{
	uint32_t uptime = Tick::GetMs();
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

