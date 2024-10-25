#pragma once

#include <zephyr/kernel.h>

typedef struct {
	int (*fopen)(void* file_ptr, const char *file_name, uint8_t mode);
	int (*fclose)(void* file_ptr);
	_ssize_t (*fwrite)(void* file_ptr, const void *ptr, size_t size);
	int (*fseek)(void* file_ptr, off_t offset, int whence);
} fit_sdk_file_mgmt_t;

void fit_sdk_init(fit_sdk_file_mgmt_t file_mgmt);
int fit_sdk_test(void* file_ptr, const char *file_name, uint8_t flags);
