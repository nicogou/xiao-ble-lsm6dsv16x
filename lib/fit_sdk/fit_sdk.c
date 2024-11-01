#include <app/lib/fit_sdk.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "fit_product.h"
#include "fit_crc.h"

LOG_MODULE_REGISTER(fit_sdk, CONFIG_FIT_SDK_LOG_LEVEL);

// 2 * PI (3.14159265)
#define TWOPI 6.2831853

// Number of semicircles per meter at the equator
#define SC_PER_M 107.173

fit_sdk_file_mgmt_t file_functions;
static FIT_UINT16 data_crc;

void WriteFileHeader(void *fp)
{
	int res;
	FIT_FILE_HDR file_header;

	file_header.header_size = FIT_FILE_HDR_SIZE;
	file_header.profile_version = FIT_PROFILE_VERSION;
	file_header.protocol_version = FIT_PROTOCOL_VERSION_20;
	memcpy((FIT_UINT8 *)&file_header.data_type, ".FIT", 4);
	res = file_functions.fseek(fp, 0, 2); /*FS_SEEK_END = 2*/
	file_header.data_size = file_functions.ftell(fp) - FIT_FILE_HDR_SIZE - sizeof(FIT_UINT16);
	file_header.crc = FitCRC_Calc16(&file_header, FIT_STRUCT_OFFSET(crc, FIT_FILE_HDR));

	res = file_functions.fseek(fp, 0, 0); /*FS_SEEK_SET = 0*/
	_ssize_t s = file_functions.fwrite(fp, (void *)&file_header, FIT_FILE_HDR_SIZE);
}

void WriteData(const void *data, FIT_UINT16 data_size, void *fp)
{
	FIT_UINT16 offset;

	_ssize_t s = file_functions.fwrite(fp, data, data_size);

	for (offset = 0; offset < data_size; offset++)
		data_crc = FitCRC_Get16(data_crc, *((FIT_UINT8 *)data + offset));
}

void WriteMessageDefinition(FIT_UINT8 local_mesg_number, const void *mesg_def_pointer, FIT_UINT16 mesg_def_size, void *fp)
{
	FIT_UINT8 header = local_mesg_number | FIT_HDR_TYPE_DEF_BIT;
	WriteData(&header, FIT_HDR_SIZE, fp);
	WriteData(mesg_def_pointer, mesg_def_size, fp);
}

void WriteMessageDefinitionWithDevFields
(
	FIT_UINT8 local_mesg_number,
	const void *mesg_def_pointer,
	FIT_UINT8 mesg_def_size,
	FIT_UINT8 number_dev_fields,
	FIT_DEV_FIELD_DEF *dev_field_definitions,
	void *fp
)
{
	FIT_UINT16 i;
	FIT_UINT8 header = local_mesg_number | FIT_HDR_TYPE_DEF_BIT | FIT_HDR_DEV_DATA_BIT;
	WriteData(&header, FIT_HDR_SIZE, fp);
	WriteData(mesg_def_pointer, mesg_def_size, fp);

	WriteData(&number_dev_fields, sizeof(FIT_UINT8), fp);
	for (i = 0; i < number_dev_fields; i++)
	{
		WriteData(&dev_field_definitions[i], sizeof(FIT_DEV_FIELD_DEF), fp);
	}
}

void WriteMessage(FIT_UINT8 local_mesg_number, const void *mesg_pointer, FIT_UINT16 mesg_size, void *fp)
{
	WriteData(&local_mesg_number, FIT_HDR_SIZE, fp);
	WriteData(mesg_pointer, mesg_size, fp);
}

void WriteDeveloperField(const void *data, FIT_UINT16 data_size, void *fp)
{
	WriteData(data, data_size, fp);
}

int fit_sdk_test(void* file_ptr, const char *file_name, uint8_t flags)
{
	int res;
	data_crc = 0;
	res = file_functions.fopen(file_ptr, file_name, flags);
	WriteFileHeader(file_ptr);

	FIT_DATE_TIME timestamp = 1000000000; // 2021-09-08T01:46:40-0600Z in seconds since the FIT Epoch of 1989-12-31T:00:00:00Z
	FIT_DATE_TIME start_time = timestamp;

	// Write file id message.
	{
		FIT_UINT8 local_mesg_number = 0;
		FIT_FILE_ID_MESG file_id_mesg;
		Fit_InitMesg(fit_mesg_defs[FIT_MESG_FILE_ID], &file_id_mesg);

		file_id_mesg.time_created = timestamp;
		file_id_mesg.type = FIT_FILE_ACTIVITY;
		file_id_mesg.manufacturer = FIT_MANUFACTURER_GARMIN;
		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_FILE_ID], FIT_FILE_ID_MESG_DEF_SIZE, file_ptr);
		WriteMessage(local_mesg_number, &file_id_mesg, FIT_FILE_ID_MESG_SIZE, file_ptr);
	}

	// Write Device Info message.
	{
		FIT_UINT8 local_mesg_number = 0;
		FIT_DEVICE_INFO_MESG device_info_mesg;
		Fit_InitMesg(fit_mesg_defs[FIT_MESG_DEVICE_INFO], &device_info_mesg);

		device_info_mesg.device_index = FIT_DEVICE_INDEX_CREATOR;
		device_info_mesg.manufacturer = FIT_MANUFACTURER_DEVELOPMENT;
		device_info_mesg.product = 0; // USE A UNIQUE ID FOR EACH OF YOUR PRODUCTS
		strcpy(device_info_mesg.product_name, "FIT Cookbook"); // Max 20 Chars
		device_info_mesg.serial_number = 123456;
		device_info_mesg.software_version = 100; // 1.0 * 100
		device_info_mesg.timestamp = timestamp;

		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_DEVICE_INFO], FIT_DEVICE_INFO_MESG_DEF_SIZE, file_ptr);
		WriteMessage(local_mesg_number, &device_info_mesg, FIT_DEVICE_INFO_MESG_SIZE, file_ptr);
	}

	// Write Event message - START Event
	{
		FIT_UINT8 local_mesg_number = 0;
		FIT_EVENT_MESG event_mesg;
		Fit_InitMesg(fit_mesg_defs[FIT_MESG_EVENT], &event_mesg);

		event_mesg.timestamp = timestamp;
		event_mesg.event = FIT_EVENT_TIMER;
		event_mesg.event_type = FIT_EVENT_TYPE_START;

		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_EVENT], FIT_EVENT_MESG_DEF_SIZE, file_ptr);
		WriteMessage(local_mesg_number, &event_mesg, FIT_EVENT_MESG_SIZE, file_ptr);
	}

	// Write Record messages.

	//The message definition only needs to be written once.
	{
		FIT_UINT8 local_mesg_number = 0;
		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_RECORD], FIT_RECORD_MESG_DEF_SIZE, file_ptr);

		for (int i = 0; i < 3600; i++)
		{
			{
				FIT_RECORD_MESG record_mesg;
				Fit_InitMesg(fit_mesg_defs[FIT_MESG_RECORD], &record_mesg);

				record_mesg.timestamp = timestamp;

				// Fake Record Data of Various Signal Patterns
				record_mesg.distance = i; // Ramp
				record_mesg.speed = 1; // Flatline
				record_mesg.heart_rate = ((FIT_UINT8)((sin(TWOPI * (0.01 * i + 10)) + 1.0) * 127.0)); // Sine
				record_mesg.cadence = ((FIT_UINT8)(i % 255)); // Sawtooth
				record_mesg.power = ((FIT_UINT16)((i % 255) < 127 ? 150 : 250)); // Square
				record_mesg.enhanced_altitude = (FIT_UINT32)(((float)fabs(((float)(i % 255)) - 127.0f)) + 500) * 5; // Triangle
				record_mesg.position_lat = 0;
				record_mesg.position_long = ((FIT_SINT32)(i * SC_PER_M));

				WriteMessage(local_mesg_number, &record_mesg, FIT_RECORD_MESG_SIZE, file_ptr);

				timestamp++;
			}
		}
	}

	// Write Event message - STOP Event
	{
		FIT_UINT8 local_mesg_number = 0;
		FIT_EVENT_MESG event_mesg;
		Fit_InitMesg(fit_mesg_defs[FIT_MESG_EVENT], &event_mesg);

		event_mesg.timestamp = timestamp;
		event_mesg.event = FIT_EVENT_TIMER;
		event_mesg.event_type = FIT_EVENT_TYPE_STOP;

		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_EVENT], FIT_EVENT_MESG_DEF_SIZE, file_ptr);
		WriteMessage(local_mesg_number, &event_mesg, FIT_EVENT_MESG_SIZE, file_ptr);
	}

	// Write Lap message.
	{
		FIT_UINT8 local_mesg_number = 0;
		FIT_LAP_MESG lap_mesg;
		Fit_InitMesg(fit_mesg_defs[FIT_MESG_LAP], &lap_mesg);

		lap_mesg.message_index = 0;
		lap_mesg.timestamp = timestamp;
		lap_mesg.start_time = start_time;
		lap_mesg.total_elapsed_time = (timestamp - start_time) * 1000;
		lap_mesg.total_timer_time = (timestamp - start_time) * 1000;

		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_LAP], FIT_LAP_MESG_DEF_SIZE, file_ptr);
		WriteMessage(local_mesg_number, &lap_mesg, FIT_LAP_MESG_SIZE, file_ptr);
	}

	// Write Session message.
	{
		FIT_UINT8 local_mesg_number = 0;
		FIT_SESSION_MESG session_mesg;
		Fit_InitMesg(fit_mesg_defs[FIT_MESG_SESSION], &session_mesg);

		session_mesg.message_index = 0;
		session_mesg.timestamp = timestamp;
		session_mesg.start_time = start_time;
		session_mesg.total_elapsed_time = (timestamp - start_time) * 1000;
		session_mesg.total_timer_time = (timestamp - start_time) * 1000;
		session_mesg.sport = FIT_SPORT_STAND_UP_PADDLEBOARDING;
		session_mesg.sub_sport = FIT_SUB_SPORT_GENERIC;
		session_mesg.first_lap_index = 0;
		session_mesg.num_laps = 1;

		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_SESSION], FIT_SESSION_MESG_DEF_SIZE, file_ptr);
		WriteMessage(local_mesg_number, &session_mesg, FIT_SESSION_MESG_SIZE, file_ptr);
	}

	// Write Activity message.
	{
		FIT_UINT8 local_mesg_number = 0;
		FIT_ACTIVITY_MESG activity_mesg;
		Fit_InitMesg(fit_mesg_defs[FIT_MESG_ACTIVITY], &activity_mesg);

		activity_mesg.timestamp = timestamp;
		activity_mesg.num_sessions = 1;
		activity_mesg.total_timer_time = (timestamp - start_time) * 1000;

		int timezoneOffset = -7 * 3600;
		activity_mesg.local_timestamp = timestamp + timezoneOffset;

		WriteMessageDefinition(local_mesg_number, fit_mesg_defs[FIT_MESG_ACTIVITY], FIT_ACTIVITY_MESG_DEF_SIZE, file_ptr);
		WriteMessage(local_mesg_number, &activity_mesg, FIT_ACTIVITY_MESG_SIZE, file_ptr);
	}

	// Write CRC.
	_ssize_t s = file_functions.fwrite(file_ptr, (void *)&data_crc, sizeof(FIT_UINT16));

	// Update file header with data size.
	WriteFileHeader(file_ptr);
	res = file_functions.fclose(file_ptr);
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
	if (!file_functions.ftell)
	{
		LOG_ERR("No file tell function defined");
	}

	LOG_DBG("FIT SDK Library");
}
