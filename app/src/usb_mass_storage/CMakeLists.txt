FILE(GLOB usb_mass_storage_sources *.c)
target_sources(app PRIVATE ${usb_mass_storage_sources})
