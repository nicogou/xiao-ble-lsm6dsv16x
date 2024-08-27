# Include the st submodule
zephyr_include_directories(lsm6dsv16x-pid)

zephyr_library()
zephyr_library_sources(lsm6dsv16x-pid/lsm6dsv16x_reg.c)
zephyr_library_sources(lsm6dsv16x.c lsm6dsv16x_sflp_utils.c)
zephyr_library_sources(platform_interface/platform_interface.c)
