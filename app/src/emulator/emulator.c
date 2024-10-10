#include "emulator.h"
#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <usb_mass_storage/usb_mass_storage.h>
#include <state_machine/state_machine.h>

LOG_MODULE_REGISTER(emulator, CONFIG_APP_LOG_LEVEL);

#define READ_SIZE 200

static emulator_sensor_t sensor;
static char* emulated_session_name = "/NAND:/SESSION1/SESSION.CSV";
static uint32_t emulated_session_waiting_time = 1000000;
static float_t ts, last_ts;
static char rd_buffer[READ_SIZE];
static uint8_t session_type = 0;

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
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
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

		emulated_session_waiting_time = (uint32_t)(1000.0 * (ts - last_ts));
		last_ts = ts;
	} else {
		LOG_ERR("Wrong number of elements! Expected %u, got %u", session_type, cnt + 1);
	}
}

void emulator_set_gbias(float_t x, float_t y, float_t z)
{
	sensor.gbias[0] = x;
	sensor.gbias[1] = y;
	sensor.gbias[2] = z;
}

void emulator_init(emulator_cb_t cb)
{
	sensor.callbacks = cb;
}

void emulator_session_start()
{
	session_type = usb_mass_storage_get_session_header(emulated_session_name, usb_mass_storage_get_session_file_p());
	ts = 0.0f;
	last_ts = 0.0f;
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
		if (session_type != 0) {
			off = usb_mass_storage_read_line(rd_buffer, READ_SIZE, 0, usb_mass_storage_get_session_file_p());
			if (off < 0)
			{
				session_type = 0;
				state_machine_post_event(XIAO_EVENT_STOP_EMULATION);
				continue;
			}
			rd_buffer[off] = 0;
			_parse_line(rd_buffer, strlen(rd_buffer));
		} else {
			off = 0;
		}
		k_usleep(emulated_session_waiting_time);
	}
}

K_THREAD_DEFINE(emulator_thread, EMULATOR_THREAD_STACK_SIZE, emulator_run, NULL, NULL, NULL, EMULATOR_THREAD_PRIORITY, 0, 0);
