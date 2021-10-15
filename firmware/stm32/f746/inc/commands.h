
#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "shell.h"
#include "nodate.h"

int shell_cmd_list(shell_cmd_args *args);
int shell_cmd_argt(shell_cmd_args *args);
int shell_cmd_deviceinfo(shell_cmd_args *args);
int shell_cmd_ping(shell_cmd_args *args);
int shell_cmd_get(shell_cmd_args *args);
int shell_cmd_enable(shell_cmd_args *args);
int shell_cmd_disable(shell_cmd_args *args);
int shell_cmd_toggle(shell_cmd_args *args);
int shell_cmd_uptime(shell_cmd_args *args);


extern shell_cmds pencil_cmds;

#endif /* __COMMANDS_H__ */
