target_sources(app PRIVATE state_machine.c)
target_sources_ifdef(CONFIG_XIAO_BLE_SHELL app PRIVATE state_machine_shell.c)
