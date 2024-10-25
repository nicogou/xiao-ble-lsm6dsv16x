#include <app/lib/fit_sdk.h>
#include <zephyr/logging/log.h>

#include "fit_product.h"
#include "fit_crc.h"

LOG_MODULE_REGISTER(fit_sdk, CONFIG_FIT_SDK_LOG_LEVEL);

fit_sdk_file_mgmt_t file_functions;

void WriteFileHeader(void *fp)
{
	int res;
	FIT_FILE_HDR file_header;

	file_header.header_size = FIT_FILE_HDR_SIZE;
	file_header.profile_version = FIT_PROFILE_VERSION;
	file_header.protocol_version = FIT_PROTOCOL_VERSION_20;
	memcpy((FIT_UINT8 *)&file_header.data_type, ".FIT", 4);
	res = file_functions.fseek(fp, 0, 2); /*FS_SEEK_END = 2*/
	LOG_DBG("seek res %i", res);
	file_header.data_size = ftell(fp) - FIT_FILE_HDR_SIZE - sizeof(FIT_UINT16);
	file_header.crc = FitCRC_Calc16(&file_header, FIT_STRUCT_OFFSET(crc, FIT_FILE_HDR));

	res = file_functions.fseek(fp, 0, 0); /*FS_SEEK_SET = 0*/
	LOG_DBG("seek res %i", res);
	_ssize_t s = file_functions.fwrite(fp, (void *)&file_header, FIT_FILE_HDR_SIZE);
	LOG_DBG("Size written: %i", s);
}

int fit_sdk_test(void* file_ptr, const char *file_name, uint8_t flags)
{
	int res;
	res = file_functions.fopen(file_ptr, file_name, flags);
	LOG_DBG("open res %i", res);
	WriteFileHeader(file_ptr);
	res = file_functions.fclose(file_ptr);
	LOG_DBG("close res %i", res);
	return 0;
}

void fit_sdk_init(fit_sdk_file_mgmt_t file_mgmt) {
	file_functions = file_mgmt;
	if (!file_functions.fopen) {
		LOG_ERR("No file open function defined");
	}
	if (!file_functions.fclose) {
		LOG_ERR("No file close function defined");
	}
	if (!file_functions.fwrite)
	{
		LOG_ERR("No file write function defined");
	}
	if (!file_functions.fseek)
	{
		LOG_ERR("No file seek function defined");
	}

	LOG_DBG("FIT SDK Library");
}
