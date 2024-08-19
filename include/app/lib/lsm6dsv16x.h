#include <zephyr/drivers/spi.h>

#define BOOT_TIME 10 //ms
#define LSM6DSV16X_SPI_OP  (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA)
#define SPI_READ		(1 << 7)

void lsm6dsv16x_init();
