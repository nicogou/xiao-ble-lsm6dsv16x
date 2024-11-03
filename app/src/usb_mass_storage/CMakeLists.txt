target_sources(app PRIVATE usb_mass_storage.c)
target_sources_ifdef(CONFIG_XIAO_BLE_SHELL app PRIVATE usb_mass_storage_shell.c)
