#include "drv8334.h"
#include <zephyr/kernel.h>

int drv8334_reg_write(const struct drv8334_config *cfg, uint8_t addr, uint16_t data)
{
    uint16_t tx_val = DRV8334_WRITE_BIT | ((addr << 11) & DRV8334_ADDR_MASK) | (data & DRV8334_DATA_MASK);
    uint16_t rx_val;

    struct spi_buf tx_buf = { .buf = &tx_val, .len = 2 };
    struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = &rx_val, .len = 2 };
    struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

    return spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
}

int drv8334_reg_read(const struct drv8334_config *cfg, uint8_t addr, uint16_t *data)
{
    uint16_t tx_val = DRV8334_READ_BIT | ((addr << 11) & DRV8334_ADDR_MASK);
    uint16_t rx_val;

    struct spi_buf tx_buf = { .buf = &tx_val, .len = 2 };
    struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = &rx_val, .len = 2 };
    struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

    int ret = spi_transceive_dt(&cfg->spi, &tx_bufs, &rx_bufs);
    if (ret == 0) {
        *data = rx_val & DRV8334_DATA_MASK;
    }
    return ret;
}

int drv8334_init(const struct drv8334_config *cfg)
{
    int ret;

    if (!spi_is_ready_dt(&cfg->spi)) return -ENODEV;

    /* Initialize GPIOs */
    gpio_pin_configure_dt(&cfg->nsleep, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&cfg->drvoff, GPIO_OUTPUT_HIGH);
    gpio_pin_configure_dt(&cfg->nfault, GPIO_INPUT);

    /* Wake up sequence */
    gpio_pin_set_dt(&cfg->nsleep, 1);
    k_msleep(2); // tWAKE duration

    /* Clear Faults */
    uint16_t stat;
    drv8334_reg_read(cfg, DRV8334_REG_FAULT_STAT_1, &stat);
    
    /* Enable Driver */
    gpio_pin_set_dt(&cfg->drvoff, 0);

    return 0;
}