#include "emulator.h"
#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <usb_mass_storage/usb_mass_storage.h>
#include <state_machine/state_machine.h>

LOG_MODULE_REGISTER(emulator, CONFIG_APP_LOG_LEVEL);

#define READ_SIZE 200
#define FILE_NAME_SIZE 40

static emulator_sensor_t sensor;
static char emulated_session_name[FILE_NAME_SIZE] = "/NAND:/SESSION1/SESSION.CSV";
static uint32_t emulated_session_waiting_time = 1000000;
static float_t ts, last_ts;
static char rd_buffer[READ_SIZE];
static int session_type = 0;

static void _parse_line(char* buf, size_t len)
{
	int32_t val[13]; // ax, ay, az, gx, gy, gz, grotx, groty, grotz, grotw, gravx, gravy, gravz;
	int8_t cnt = -1;
	char *pt;
    pt = strtok (buf,",");
    while (pt != NULL) {
		cnt++;
        switch (cnt)
		{
		case 0:
			ts = strtof(pt, NULL);
			break;
		case 1 ... 13:
			val[cnt-1] = strtol(pt, NULL, 10);
			break;

		default:
			LOG_WRN("Unhandled item in session");
			break;
		}
        pt = strtok (NULL, ",");
    }
	if (session_type == cnt + 1)
	{
		// Call callback functions.
		if (sensor.callbacks.emulator_ts_sample_cb)
		{
			(*sensor.callbacks.emulator_ts_sample_cb)(ts);
		}
		if (sensor.callbacks.emulator_acc_sample_cb)
		{
			(*sensor.callbacks.emulator_acc_sample_cb)((float_t)val[0], (float_t)val[1], (float_t)val[2]);
		}
		if (sensor.callbacks.emulator_gyro_sample_cb)
		{
			(*sensor.callbacks.emulator_gyro_sample_cb)((float_t)val[3], (float_t)val[4], (float_t)val[5]);
		}
		if (session_type == EMULATOR_SESSION_HEADER_SFLP)
		{
			// Only call these functions if needed.
			if (sensor.callbacks.emulator_game_rot_sample_cb)
			{
				(*sensor.callbacks.emulator_game_rot_sample_cb)((float_t)val[6], (float_t)val[7], (float_t)val[8], (float_t)val[9]);
			}
			if (sensor.callbacks.emulator_gravity_sample_cb)
			{
				(*sensor.callbacks.emulator_gravity_sample_cb)((float_t)val[10], (float_t)val[11], (float_t)val[12]);
			}
		}

		emulated_session_waiting_time = (uint32_t)(1000.0f * (ts - last_ts));
		last_ts = ts;
	} else {
		LOG_ERR("Wrong number of elements! Expected %u, got %u", session_type, cnt + 1);
	}
}

int emulator_set_session(char* file_path)
{
	if (strlen(file_path) > FILE_NAME_SIZE) {
		LOG_ERR("File name too long");
		return -E2BIG;
	}
	strcpy(emulated_session_name, file_path);
	return 0;
}

void emulator_init(emulator_cb_t cb)
{
	sensor.callbacks = cb;
}

int emulator_session_start()
{
	LOG_INF("Emulating file %s", emulated_session_name);
	session_type = usb_mass_storage_get_session_header(emulated_session_name, usb_mass_storage_get_session_file_p());
	ts = 0.0f;
	last_ts = 0.0f;
	if (session_type < 0)
	{
		LOG_ERR("Unable to start emulation (%i)", session_type);
		return session_type;
	}

	if (session_type == EMULATOR_SESSION_HEADER_SFLP)
	{
		xiao_recording_state_t recording_state = state_machine_get_recording_state();
		recording_state.sflp_enabled = true;
		state_machine_set_recording_state(recording_state);
		LOG_INF("Emulated session has SFLP enabled!");
	}

	return session_type;
}

void emulator_session_stop()
{
	emulated_session_waiting_time = 1000000; // When not emulating, wait 1s at a time.
	session_type = 0;
	usb_mass_storage_close_file(usb_mass_storage_get_session_file_p());
}

static void emulator_run(void *p1, void *p2, void *p3)
{
	int off = 0;

	while(true) {
		if (session_type > 0) {
			off = usb_mass_storage_read_line(rd_buffer, READ_SIZE, 0, usb_mass_storage_get_session_file_p());
			if (off < 0)
			{
				session_type = 0;
				state_machine_post_event(XIAO_EVENT_STOP_RECORDING);
				continue;
			}
			rd_buffer[off] = 0;
			_parse_line(rd_buffer, strlen(rd_buffer));
		} else {
			if (session_type < 0) {
				state_machine_post_event(XIAO_EVENT_STOP_RECORDING);
				session_type = 0;
			}
			off = 0;
		}
		k_usleep(emulated_session_waiting_time);
	}
}

K_THREAD_DEFINE(emulator_thread, EMULATOR_THREAD_STACK_SIZE, emulator_run, NULL, NULL, NULL, EMULATOR_THREAD_PRIORITY, 0, 0);
