#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>

#define LSM6DSV16X_SPI_OP  (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA)
#define SPI_READ		(1 << 7)

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
void platform_delay(uint32_t ms);
void platform_init(void *handle);
