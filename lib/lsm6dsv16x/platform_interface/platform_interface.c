#include "platform_interface.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_interface, CONFIG_LSM6DSV16X_LOG_LEVEL);

// Configuration structures
const struct device *imu_spi = DEVICE_DT_GET(DT_NODELABEL(xiao_spi));
const struct spi_config imu_spi_cfg = {
    .frequency = DT_PROP(DT_NODELABEL(spi2), clock_frequency),
    .operation = LSM6DSV16X_SPI_OP,
    .cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(lsm6dsv16x), 10),
};

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

    return spi_write(imu_spi, &imu_spi_cfg, &tx);
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


    return spi_transceive(imu_spi, &imu_spi_cfg, &tx, &rx);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
void platform_init(void *handle)
{
	// Nothing to do here for Xiao BLE Sense
}
