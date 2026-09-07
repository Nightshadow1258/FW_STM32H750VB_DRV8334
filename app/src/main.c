/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate AS5047P magnetic encoder driver.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/pwm.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/spi.h>

#include <zephyr/drivers/sensor.h>

#include "as5047p.h"
#include "storage_manager.h"
#include "adc_manager.h"
#include "drv8334.h"
#include "pwm_timer.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define SPI3_NODE DT_NODELABEL(spi3)
static const struct device *dev_mag_as5047p = DEVICE_DT_GET(SPI3_NODE);

static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(as5047p_cs), gpios);

static struct spi_config spi_cfg = {
    .frequency = 5000000, // 500 KHz
    .operation = SPI_OP_MODE_MASTER |
                 SPI_WORD_SET(8) |
                 SPI_TRANSFER_MSB |
                 SPI_MODE_CPHA,
    .slave = 0,
    .cs = {NULL}, // <--- If this is set, CS is handled by the hardware of the MCU, NULL not connected manual mode
};

// ABI Interface
/* Get the devices from Devicetree */
const struct device *qdec_dev = DEVICE_DT_GET(DT_NODELABEL(qdec));
static const struct gpio_dt_spec index_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(index_pin), gpios);
static struct gpio_callback index_cb_data;

/* Callback function for Index pulse */
void index_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    /* Reset the timer counter when index pulse is detected */
    /* Note: Direct register access is often fastest for this */
    TIM2->CNT = 0;
}

void setup_encoder(void)
{
    if (!device_is_ready(qdec_dev))
    {
        return;
    }

    /* Setup Index Interrupt */
    gpio_pin_configure_dt(&index_gpio, GPIO_INPUT);
    gpio_init_callback(&index_cb_data, index_handler, BIT(index_gpio.pin));
    gpio_add_callback(index_gpio.port, &index_cb_data);
    gpio_pin_interrupt_configure_dt(&index_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

double read_encoder(void)
{
    struct sensor_value qdec_value = {0};
    double precise_degrees = 0.0;
    if (sensor_sample_fetch(qdec_dev) == 0)
    {
        sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &qdec_value);

        // Convert to double to see the 0.08789 degree increments
        precise_degrees = sensor_value_to_double(&qdec_value);
    }
    return precise_degrees;
}

// Measurement

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

// Trigger pin for starting the measurement
static const struct gpio_dt_spec trigger_gpio =
    GPIO_DT_SPEC_GET(DT_NODELABEL(meas_trigger), gpios);

static struct gpio_callback trigger_cb_data;
static K_SEM_DEFINE(meas_sem, 0, 1);

static uint32_t last_trigger_time = 0;
#define DEBOUNCE_TIMEOUT_MS 100
void trigger_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    uint32_t now = k_uptime_get_32();

    // Only process the interrupt if enough time has passed
    if ((now - last_trigger_time) > DEBOUNCE_TIMEOUT_MS)
    {
        last_trigger_time = now;
        k_sem_give(&meas_sem);
    }
}

/* Data Structure for high-speed capture */
struct sensor_test_sample
{
    uint32_t timestamp;
    float abi_ticks;
    uint16_t spi_raw;
};

/* Buffer configuration */
#define MAX_SAMPLES 4096 // max ist around 8000
static struct sensor_test_sample sample_buffer[MAX_SAMPLES];

void dump_characterization_data(void)
{
    // Wait for the measurement thread to be 100% idle
    k_msleep(50);

    // Get the hardware clock frequency
    uint32_t hw_freq = sys_clock_hw_cycles_per_sec();

    // Capture the first timestamp as the zero reference
    uint32_t start_cycles = sample_buffer[0].timestamp;

    // Use a simpler header
    printk("idx,ts [ms],Angle from ABI [°],Angle from SPI [°],Angle Linearization [°]\n");

    for (uint32_t i = 0; i < MAX_SAMPLES; i++)
    {
        // 1. Calculate relative cycles from the start
        uint32_t rel_cycles = sample_buffer[i].timestamp - start_cycles;

        /* Calculate ms as float then round to nearest integer */
        float rel_ms_float = (float)rel_cycles / (hw_freq / 1000.0f);
        uint32_t rel_ms_int = (uint32_t)(rel_ms_float + 0.5f);

        float linear_corrected = apply_linear_correction(sample_buffer[i].abi_ticks);

        // Print and then wait for the UART to clear
        printk("%u,%u,%.4f,%.4f,%.4f\n",
               i,
               rel_ms_int,
               (double)sample_buffer[i].abi_ticks,
               (double)sample_buffer[i].spi_raw * (360.0 / 16384.0),
               (double)linear_corrected);

        // Slow down the dump significantly to ensure no data loss
        // 3ms delay per line is safe for 115200 baud
        k_msleep(3);
    }
}

/* Thread function */
void measurement_thread_entry(void *p1, void *p2, void *p3)
{
    struct k_timer timer;
    k_timer_init(&timer, NULL, NULL);

    while (1)
    {
        /* Wait for PB3 trigger */
        k_sem_take(&meas_sem, K_FOREVER);

        /* Optional 100ms startup delay */
        // k_msleep(100);
        printk("Measurement triggered! Starting data capture...\n");
        uint32_t start_time = k_uptime_get_32();

        /* Start the interval timer for 50ms */
        k_timer_start(&timer, K_NO_WAIT, K_MSEC(2.5)); // 2.5ms interval for 400Hz sampling

        /* Collect exactly MAX_SAMPLES */
        for (int i = 0; i < MAX_SAMPLES; i++)
        {
            k_timer_status_sync(&timer);
            /* 1. Fetch ABI Ticks */
            struct sensor_value qdec_value = {0};
            double precise_degrees = 0.0;
            if (sensor_sample_fetch(qdec_dev) == 0)
            {
                sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &qdec_value);

                // Convert to double to see the 0.08789 degree increments
                precise_degrees = sensor_value_to_double(&qdec_value);
            }

            /* 2. Fetch SPI Raw Angle */
            uint16_t raw_reg = as5047p_read_reg(dev_mag_as5047p, &spi_cfg, &cs_gpio, AS5047P_REG_ANGLECOM);
            /* 3. Store to RAM using the loop index 'i' */
            sample_buffer[i].timestamp = k_cycle_get_32();
            sample_buffer[i].abi_ticks = precise_degrees;
            sample_buffer[i].spi_raw = raw_reg & 0x3FFF;
            /* Wait for the remainder of the 10ms interval */
        }
        k_timer_stop(&timer);

        uint32_t end_time = k_uptime_get_32();
        printk("Capture End. Duration: %u ms\n", (end_time - start_time));
        printk("Data capture complete! Dumping data...\n");
        dump_characterization_data();
        k_sem_reset(&meas_sem);
        /* Thread automatically loops back to k_sem_take and sleeps */
    }
}
K_THREAD_DEFINE(meas_thread_id, 2048, measurement_thread_entry, NULL, NULL, NULL, 2, 0, 0);

// flash
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#define STORAGE_NODE DT_ALIAS(storage_flash)

int main(void)
{
    if (!gpio_is_ready_dt(&cs_gpio))
    {
        LOG_ERR("CS GPIO device not ready");
        return -ENODEV;
    }
    /* Configure as output and set to High (Inactive) */
    gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(dev_mag_as5047p))
    {
        return -ENODEV;
    }

    // Setup the encoder pins for the ABI Interface
    setup_encoder();

    // Check register settings
    as5047p_dump_registers(dev_mag_as5047p, &spi_cfg, &cs_gpio);
    as5047p_set_abi_4096(dev_mag_as5047p, &spi_cfg, &cs_gpio);
    uint16_t spi_raw = as5047p_read_reg(dev_mag_as5047p, &spi_cfg, &cs_gpio, AS5047P_REG_ANGLECOM);
    spi_raw &= 0x3FFF; // Mask to 14 bits

    // 2. Scale 14-bit (16384) to 12-bit (4096)
    // Using 4096.0 for float precision or simple shift for speed
    uint32_t initial_counts = spi_raw / 8;

    // 3. Directly set the STM32 Timer Counter register
    // TIM2 is used for ABI
    TIM2->CNT = initial_counts;
    as5047p_dump_registers(dev_mag_as5047p, &spi_cfg, &cs_gpio);
    k_msleep(100);

    // Interrupt setup for measurement trigger
    gpio_pin_configure_dt(&trigger_gpio, GPIO_INPUT);

    /* Configure interrupt for Rising Edge (Low to High) */
    gpio_pin_interrupt_configure_dt(&trigger_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    /* Attach the callback */
    gpio_init_callback(&trigger_cb_data, trigger_handler, BIT(trigger_gpio.pin));
    gpio_add_callback(trigger_gpio.port, &trigger_cb_data);

    printk("INFO: System Ready. Trigger enabled.\n");

    storage_run_test();

    start_center_aligned_3phase_pwm();
    printk("INFO: Timer 1 center aligned 3 phase pwm started.\n");

    // DRV8334 Initialization

    int ret = start_timer_triggered_adc_isr();
    if (ret < 0)
    {
        printk("Failed to start ADC ISR mode: %d\n", ret);
        return ret;
    }

    printk("ADC ISR system initialized. Awaiting TIM1 TRGO events...\n");

    ret = drv8334_init(&drv_cfg);
    if (ret < 0)
    {
        printk("INFO: DRV8334 Initialization Failed: %d\n", ret);
    }
    else
    {
        printk("INFO: DRV8334 Initialized successfully\n");
    }

    printk("INFO: System Testing Active. Ready to sample signals.\n");

    while (1)
    {
        k_sleep(K_FOREVER);
    }

    return 0;
}