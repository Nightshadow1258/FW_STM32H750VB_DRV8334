#ifndef DRV8334_H
#define DRV8334_H

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

/* SPI Command bits */
#define DRV8334_READ_BIT                0x8000
#define DRV8334_WRITE_BIT               0x0000
#define DRV8334_ADDR_MASK               0x7800
#define DRV8334_DATA_MASK               0x07FF

/* Register Addresses */
#define DRV8334_REG_FAULT_STAT_1        0x00
#define DRV8334_REG_FAULT_STAT_2        0x01
#define DRV8334_REG_GDRV_CTRL_1         0x02
#define DRV8334_REG_GDRV_CTRL_2         0x03
#define DRV8334_REG_GDRV_CTRL_3         0x04
#define DRV8334_REG_GDRV_CTRL_4         0x05
#define DRV8334_REG_CSA_CTRL            0x06
#define DRV8334_REG_OCP_CTRL            0x07

/* Configuration Masks (Example) */
#define DRV8334_GDRV_CTRL_1_PWM_6X      0x0000
#define DRV8334_GDRV_CTRL_1_PWM_3X      0x0020

struct drv8334_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec nsleep;
    struct gpio_dt_spec drvoff;
    struct gpio_dt_spec nfault;
};

int drv8334_init(const struct drv8334_config *cfg);
int drv8334_reg_write(const struct drv8334_config *cfg, uint8_t addr, uint16_t data);
int drv8334_reg_read(const struct drv8334_config *cfg, uint8_t addr, uint16_t *data);

#endif