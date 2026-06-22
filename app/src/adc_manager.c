/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_manager.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

/* Map indexes from the io-channels devicetree layout array */
static const struct adc_dt_spec adc_soa  = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_sob  = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct adc_dt_spec adc_soc  = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);
//static const struct adc_dt_spec adc_temp = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3);

static struct motor_current_samples latest_currents;
static bool adc_ready = false;

int adc_manager_init(void)
{
    int rc;

    if (!device_is_ready(adc_soa.dev) || !device_is_ready(adc_sob.dev) || 
        !device_is_ready(adc_soc.dev))  { //|| !device_is_ready(adc_temp.dev))
        printk("Error: One or more physical ADC hardware units are not ready\n");
        return -ENODEV;
    }

    /* Set up internal channel configurations for each peripheral */
    rc = adc_channel_setup_dt(&adc_soa);
    rc |= adc_channel_setup_dt(&adc_sob);
    rc |= adc_channel_setup_dt(&adc_soc);
    //rc |= adc_channel_setup_dt(&adc_temp);
    
    if (rc < 0) {
        printk("Error: Channel internal assignment configuration failed\n");
        return rc;
    }

    adc_ready = true;
    printk("ADC Manager initialized. Channels 3x Current + 1x Temperature active.\n");
    return 0;
}

int adc_manager_read_currents(struct motor_current_samples *out_samples)
{
    if (!adc_ready || out_samples == NULL) {
        return -EAGAIN;
    }

    /* Buffers for each hardware instance */
    int16_t buf_a, buf_b, buf_c;

    struct adc_sequence seq_a = { .buffer = &buf_a, .buffer_size = sizeof(buf_a) };
    struct adc_sequence seq_b = { .buffer = &buf_b, .buffer_size = sizeof(buf_b) };
    struct adc_sequence seq_c = { .buffer = &buf_c, .buffer_size = sizeof(buf_c) };

    adc_sequence_init_dt(&adc_soa, &seq_a);
    adc_sequence_init_dt(&adc_sob, &seq_b);
    adc_sequence_init_dt(&adc_soc, &seq_c);

    /* * Executing reads in direct succession. For absolute simultaneous hardware 
     * sampling, later enhancements can configure an external hardware timer trigger 
     * (e.g., TIM1/TIM8 TRGO) via direct HAL registers.
     */
    int rc_a = adc_read(adc_soa.dev, &seq_a);
    int rc_b = adc_read(adc_sob.dev, &seq_b);
    int rc_c = adc_read(adc_soc.dev, &seq_c);

    if (rc_a < 0 || rc_b < 0 || rc_c < 0) {
        return -EIO;
    }

    latest_currents.raw_a = buf_a;
    latest_currents.raw_b = buf_b;
    latest_currents.raw_c = buf_c;

    /* Process raw digits to electrical millivolts */
    latest_currents.mv_a = (int32_t)buf_a;
    latest_currents.mv_b = (int32_t)buf_b;
    latest_currents.mv_c = (int32_t)buf_c;

    adc_raw_to_millivolts_dt(&adc_soa, &latest_currents.mv_a);
    adc_raw_to_millivolts_dt(&adc_sob, &latest_currents.mv_b);
    adc_raw_to_millivolts_dt(&adc_soc, &latest_currents.mv_c);

    /* Output copy back to thread target structure */
    *out_samples = latest_currents;

    return 0;
}

// int adc_manager_read_temp(int32_t *out_mv)
// {
//     if (!adc_ready || out_mv == NULL) {
//         return -EAGAIN;
//     }

//     int16_t buf_t;
//     struct adc_sequence seq_t = { .buffer = &buf_t, .buffer_size = sizeof(buf_t) };
//     adc_sequence_init_dt(&adc_temp, &seq_t);

//     int rc = adc_read(adc_temp.dev, &seq_t);
//     if (rc < 0) {
//         return rc;
//     }

//     int32_t mv = (int32_t)buf_t;
//     rc = adc_raw_to_millivolts_dt(&adc_temp, &mv);
//     if (rc < 0) {
//         return rc;
//     }

//     *out_mv = mv;
//     return 0;
// }