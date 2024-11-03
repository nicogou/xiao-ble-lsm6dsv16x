target_sources(app PRIVATE battery.c)
target_sources_ifdef(CONFIG_XIAO_BLE_SHELL app PRIVATE battery_shell.c)
