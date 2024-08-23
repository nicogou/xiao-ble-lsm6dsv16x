FILE(GLOB state_machine_sources *.c)
target_sources(app PRIVATE ${state_machine_sources})
