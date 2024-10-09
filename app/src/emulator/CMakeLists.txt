FILE(GLOB emulator_sources *.c)
target_sources(app PRIVATE ${emulator_sources})
