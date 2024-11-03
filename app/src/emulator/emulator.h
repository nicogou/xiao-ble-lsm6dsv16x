#include <math.h>
#include <zephyr/kernel.h>

#define EMULATOR_THREAD_STACK_SIZE 1024
#define EMULATOR_THREAD_PRIORITY 5
#define EMULATOR_SESSION_HEADER_SIMPLE 7
#define EMULATOR_SESSION_HEADER_SFLP 14

typedef struct {
	void (*emulator_ts_sample_cb)(float_t);
	void (*emulator_acc_sample_cb)(float_t, float_t, float_t);
	void (*emulator_gyro_sample_cb)(float_t, float_t, float_t);
	void (*emulator_game_rot_sample_cb)(float_t, float_t, float_t, float_t);
	void (*emulator_gravity_sample_cb)(float_t, float_t, float_t);
} emulator_cb_t;

typedef struct {
	emulator_cb_t callbacks;
} emulator_sensor_t;

void emulator_init(emulator_cb_t cb);
int emulator_set_session(char* file_path);
int emulator_session_start();
void emulator_session_stop();
