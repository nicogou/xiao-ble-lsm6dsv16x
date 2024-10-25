#include <app/lib/fit_sdk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fit_sdk, CONFIG_FIT_SDK_LOG_LEVEL);

void fit_sdk_init() {
	LOG_DBG("FIT SDK Library");
}
