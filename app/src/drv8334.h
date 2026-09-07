#ifndef DRV8334_H
#define DRV8334_H

#include <stdlib.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>


#include <zephyr/kernel.h>
#include <stm32h7xx_hal.h>


/* SPI Protocol Definitions */
#define DRV8334_READ_BIT                0x8000
#define DRV8334_WRITE_BIT               0x0000
#define DRV8334_ADDR_MASK               0x7800
#define DRV8334_DATA_MASK               0x07FF

/* ================= STATUS REGISTERS (0x00 - 0x05) ================= */
#define DRV8334_REG_IC_STAT1            0x00
#define DRV8334_REG_IC_STAT2            0x01
#define DRV8334_REG_IC_STAT3            0x02
#define DRV8334_REG_IC_STAT4            0x03
#define DRV8334_REG_IC_STAT5            0x04
#define DRV8334_REG_IC_STAT6            0x05

/* ================= CONTROL REGISTERS (0x1A - 0x48) ================= */
#define DRV8334_REG_IC_CTRL1            0x1A
#define DRV8334_REG_IC_CTRL2            0x1B
#define DRV8334_REG_IC_CTRL3            0x1C
#define DRV8334_REG_GD_CTRL1            0x1E
#define DRV8334_REG_GD_CTRL2            0x1F
#define DRV8334_REG_GD_CTRL3            0x21
#define DRV8334_REG_GD_CTRL3B           0x22
#define DRV8334_REG_GD_CTRL4            0x23
#define DRV8334_REG_GD_CTRL5            0x24
#define DRV8334_REG_GD_CTRL6            0x25
#define DRV8334_REG_GD_CTRL7            0x26
#define DRV8334_REG_CSA_CTRL            0x29
#define DRV8334_REG_MON_CTRL1           0x2B
#define DRV8334_REG_MON_CTRL2           0x2C
#define DRV8334_REG_MON_CTRL3           0x2D
#define DRV8334_REG_MON_CTRL4           0x2E
#define DRV8334_REG_MON_CTRL5           0x2F
#define DRV8334_REG_MON_CTRL6           0x30
#define DRV8334_REG_DIAG_CTRL1          0x33
#define DRV8334_REG_SPI_TEST            0x36
#define DRV8334_REG_OTP_USR             0x48

/* Configuration Masks (Example) */
#define DRV8334_GDRV_CTRL_1_PWM_6X      0x0000
#define DRV8334_GDRV_CTRL_1_PWM_3X      0x0020

// Timer and Dead-Time Configuration for STM32H750 (Core @ 480 MHz, APB2 Timers @ 240 MHz)
#define TIM_CLOCK_HZ      240000000
#define PWM_FREQ_HZ       20000
#define TIM_PERIOD_TICKS (TIM_CLOCK_HZ / (2 * PWM_FREQ_HZ))
#define DEAD_TIME_NS      500 /* 500ns dead time for DRV8334 */
#define DEAD_TIME_TICKS   ((uint32_t)((DEAD_TIME_NS * 240) / 1000)) /* ~120 ticks */


extern TIM_HandleTypeDef htim1;

struct drv8334_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec nsleep;
    struct gpio_dt_spec drvoff;
    struct gpio_dt_spec nfault;
};

extern const struct drv8334_config drv_cfg;

int drv8334_init(const struct drv8334_config *cfg);
int drv8334_reg_write(const struct drv8334_config *cfg, uint8_t addr, uint16_t data);
int drv8334_reg_read(const struct drv8334_config *cfg, uint8_t addr, uint16_t *data);

int drv8334_dump_status_regs(const struct drv8334_config *cfg, const struct shell *sh);
int drv8334_dump_control_regs(const struct drv8334_config *cfg, const struct shell *sh);

void init_tim1_foc(void); 


#endif