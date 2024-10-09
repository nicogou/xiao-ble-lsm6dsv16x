#include "emulator.h"
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <usb_mass_storage/usb_mass_storage.h>
#include <state_machine/state_machine.h>

LOG_MODULE_REGISTER(emulator, CONFIG_APP_LOG_LEVEL);

#define READ_SIZE 200

static emulator_sensor_t sensor;
static char* emulated_session_name = "/NAND:/SESSION1/SESSION.CSV";
static uint32_t emulated_session_start_time = 0;
static char rd_buffer[READ_SIZE];
static uint8_t session_type = 0;

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

void emulator_session_start(){
	emulated_session_start_time = k_uptime_get_32();

	session_type = usb_mass_storage_get_session_header(emulated_session_name, usb_mass_storage_get_session_file_p());
}

void emulator_session_stop()
{
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
			}
			rd_buffer[off + 1] = 0;
			LOG_HEXDUMP_DBG(rd_buffer, off, "read buffer");
		} else {
			off = 0;
		}
		k_msleep(33);
	}
}

K_THREAD_DEFINE(emulator_thread, EMULATOR_THREAD_STACK_SIZE, emulator_run, NULL, NULL, NULL, EMULATOR_THREAD_PRIORITY, 0, 0);
