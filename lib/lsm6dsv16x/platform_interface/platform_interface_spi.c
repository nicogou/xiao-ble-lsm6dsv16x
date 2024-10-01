#include "platform_interface.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_interface, CONFIG_LSM6DSV16X_LOG_LEVEL);
// Configuration structures
static const struct spi_dt_spec imu_spi = SPI_DT_SPEC_GET(DT_NODELABEL(lsm6dsv16x), LSM6DSV16X_SPI_OP, 10);

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
	uint8_t buffer_tx[1] = { reg & ~SPI_READ };
        /*
	 *   transaction #1: write 1 byte with reg addr (msb at 0)
	 *   transaction #2: write "len" byte of data
	 */
	const struct spi_buf tx_buf[2] = {
		{ .buf = buffer_tx, .len = 1, },
		{ .buf = bufp, .len = len, }
	};
	const struct spi_buf_set tx = { .buffers = tx_buf, .count = 2 };
        return spi_write_dt(&imu_spi, &tx);
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
	uint8_t buffer_tx[2] = { reg | SPI_READ, 0 };
	/*  write 1 byte with reg addr (msb at 1) + 1 dummy byte */
	const struct spi_buf tx_buf = { .buf = buffer_tx, .len = 2, };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	/*
	 *   transaction #1: dummy read to skip first byte
	 *   transaction #2: read "len" byte of data
	 */
	const struct spi_buf rx_buf[2] = {
		{ .buf = NULL, .len = 1, },
		{ .buf = bufp, .len = len, }
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = 2 };
        return spi_transceive_dt(&imu_spi, &tx, &rx);
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
