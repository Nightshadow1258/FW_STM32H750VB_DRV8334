/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADC_MANAGER_H_
#define ADC_MANAGER_H_

#include <zephyr/types.h>

/**
 * @brief Structure holding the latest raw and converted millivolt data.
 * Ready for extraction by control feedback or background loops.
 */
struct motor_current_samples {
    int16_t raw_a;
    int16_t raw_b;
    int16_t raw_c;
    int32_t mv_a;
    int32_t mv_b;
    int32_t mv_c;
};

/**
 * @brief Initializes all three physical ADC channels and sets up structures.
 * @return 0 on success, negative error code on failure.
 */
int adc_manager_init(void);

/**
 * @brief Performs a synchronized read of Phase A, B, and C currents.
 * Stores the values internally for immediate control-loop retrieval.
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

#endif /* ADC_MANAGER_H_ */