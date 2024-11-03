# Include the FIT SDK submodule
zephyr_include_directories(custom_fit_sdk/c)

zephyr_library()

# Add FIT SDK files
zephyr_library_sources(
	custom_fit_sdk/c/fit.c
	custom_fit_sdk/c/fit_ram.c
	custom_fit_sdk/c/fit_product.c
	custom_fit_sdk/c/fit_crc.c
	custom_fit_sdk/c/fit_convert.c
)

# Add library wrapper
zephyr_library_sources(fit_sdk.c)
