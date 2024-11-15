#include <zephyr/kernel.h>

typedef struct {
	void (*on_connection_success)();
	void (*on_connection_fail)(uint8_t);
	void (*on_disconnection)(uint8_t);
} xiao_smp_bluetooth_cb_t;

void smp_bluetooth_init(xiao_smp_bluetooth_cb_t cb);
int smp_bluetooth_start_advertising();
int smp_bluetooth_stop_advertising();
