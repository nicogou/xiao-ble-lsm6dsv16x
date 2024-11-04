#include <zephyr/kernel.h>

typedef struct {
	void (*on_connection_success)();
	void (*on_connection_fail)(uint8_t);
	void (*on_disconnection)(uint8_t);
} xiao_smp_bluetooth_cb_t;

void start_smp_bluetooth_adverts(xiao_smp_bluetooth_cb_t cb);
