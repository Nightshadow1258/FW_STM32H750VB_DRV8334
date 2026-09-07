/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADC_MANAGER_H_
#define ADC_MANAGER_H_

#include <stdlib.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stm32h7xx_hal.h>
#include <zephyr/arch/cpu.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_system.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include <zephyr/drivers/pinctrl.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_adc_ex.h>

#include <stm32h7xx_hal_dma.h>

#include <zephyr/irq.h>
#include <stm32_ll_adc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure holding the latest raw and converted millivolt data.
 * Ready for extraction by control feedback or background loops.
 */
struct motor_current_sample {
    uint16_t raw_a;
    uint16_t raw_b;
    uint16_t raw_c;
    int32_t mv_a;
    int32_t mv_b;
    int32_t mv_c;
    int32_t c_a;
    int32_t c_b;
    int32_t c_c;
};

/**
 * @brief Initializes ADC1, ADC2, and ADC3 for simultaneous triple injected
 * sampling triggered by TIM1.
 */
void init_adcs_simultaneous_foc(void);

/**
 * @brief Returns the initialization status of the ADC subsystem.
 * @return true if initialized, false otherwise.
 */
bool is_adc_ready(void);

/**
 * @brief Thread-safe retrieval of the latest phase current measurements.
 * @param samples Pointer to output structure to receive current data.
 */
void get_latest_currents(struct motor_current_samples *samples);

/**
 * @brief Performs a synchronized read of Phase A, B, and C currents.
 * @param out_samples Pointer to output structure to copy the latest readings.
 * @return 0 on success, negative error code on failure.
 */
int adc_manager_read_currents(struct motor_current_samples *out_samples);

/**
 * @brief Performs a low-priority synchronous sample of the NTC channel.
 * @param out_mv Pointer to store the raw voltage in millivolts.
 * @return 0 on success, negative error code on failure.
 */
int adc_manager_read_temp(int32_t *out_mv);

/**
 * @brief Prints hardware register diagnostics for a designated ADC instance and channel.
 * @param adc_num ADC peripheral instance (1, 2, or 3).
 * @param channel Channel number (0 to 19).
 */
void print_adc_channel_debug(uint8_t adc_num, uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif /* ADC_MANAGER_H_ */