#include "platform_interface.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(platform_interface, CONFIG_LSM6DSV16X_LOG_LEVEL);

static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(lsm6dsv16x));
/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms)
{
	k_msleep(ms);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	return i2c_burst_write_dt(&imu_i2c, reg, bufp, len);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	return i2c_burst_read_dt(&imu_i2c, reg, bufp, len);
}

int8_t attach_interrupt(const struct gpio_dt_spec gpio, gpio_flags_t input, gpio_flags_t edge, struct gpio_callback *callback, gpio_callback_handler_t handler)
{
        int8_t ret;

        /* gpio pin setup */
        if (!device_is_ready(gpio.port))
        {
                LOG_ERR("Error: button device %s is not ready",
                        gpio.port->name);
                return -1;
        }

        ret = gpio_pin_configure_dt(&gpio, input);
        if (ret != 0)
        {
                LOG_ERR("Error %d: failed to configure %s pin %d",
                        ret, gpio.port->name, gpio.pin);
                return -2;
        }

        ret = gpio_pin_interrupt_configure_dt(&gpio, edge);
        if (ret != 0)
        {
                LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
                        ret, gpio.port->name, gpio.pin);
                return -3;
        }

        gpio_init_callback(callback, handler, BIT(gpio.pin));
        gpio_add_callback(gpio.port, callback);

        return 0;
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
void platform_init(void *handle)
{
	// Nothing to do here for Xiao BLE Sense
}
