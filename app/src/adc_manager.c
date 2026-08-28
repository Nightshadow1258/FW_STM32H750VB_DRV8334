/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_manager.h"


/* Map indexes from the io-channels devicetree layout array */
static const struct adc_dt_spec adc_soa = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_sob = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct adc_dt_spec adc_soc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);
// static const struct adc_dt_spec adc_temp = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3);

static struct motor_current_samples latest_currents;
static bool adc_ready = false;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

int adc_manager_init(void)
{
    int rc;

    if (!device_is_ready(adc_soa.dev) || !device_is_ready(adc_sob.dev) ||
        !device_is_ready(adc_soc.dev))
    { //|| !device_is_ready(adc_temp.dev))
        printk("Error: One or more physical ADC hardware units are not ready\n");
        return -ENODEV;
    }

    /* Set up internal channel configurations for each peripheral */
    rc = adc_channel_setup_dt(&adc_soa);
    rc |= adc_channel_setup_dt(&adc_sob);
    rc |= adc_channel_setup_dt(&adc_soc);
    // rc |= adc_channel_setup_dt(&adc_temp);

    if (rc < 0)
    {
        printk("Error: Channel internal assignment configuration failed\n");
        return rc;
    }

    adc_ready = true;
    printk("ADC Manager initialized. Channels 3x Current + 1x Temperature active.\n");
    return 0;
}

// int adc_manager_read_currents(struct motor_current_samples *out_samples)
// {
//     if (!adc_ready || out_samples == NULL) {
//         return -EAGAIN;
//     }

//     /* Buffers for each hardware instance */
//     int16_t buf_a, buf_b, buf_c;

//     struct adc_sequence seq_a = { .buffer = &buf_a, .buffer_size = sizeof(buf_a) };
//     struct adc_sequence seq_b = { .buffer = &buf_b, .buffer_size = sizeof(buf_b) };
//     struct adc_sequence seq_c = { .buffer = &buf_c, .buffer_size = sizeof(buf_c) };

//     adc_sequence_init_dt(&adc_soa, &seq_a);
//     adc_sequence_init_dt(&adc_sob, &seq_b);
//     adc_sequence_init_dt(&adc_soc, &seq_c);

//     /* * Executing reads in direct succession. For absolute simultaneous hardware
//      * sampling, later enhancements can configure an external hardware timer trigger
//      * (e.g., TIM1/TIM8 TRGO) via direct HAL registers.
//      */
//     int rc_a = adc_read(adc_soa.dev, &seq_a);
//     int rc_b = adc_read(adc_sob.dev, &seq_b);
//     int rc_c = adc_read(adc_soc.dev, &seq_c);

//     if (rc_a < 0 || rc_b < 0 || rc_c < 0) {
//         return -EIO;
//     }

//     latest_currents.raw_a = buf_a;
//     latest_currents.raw_b = buf_b;
//     latest_currents.raw_c = buf_c;

//     /* Process raw digits to electrical millivolts */
//     latest_currents.mv_a = (int32_t)buf_a;
//     latest_currents.mv_b = (int32_t)buf_b;
//     latest_currents.mv_c = (int32_t)buf_c;

//     adc_raw_to_millivolts_dt(&adc_soa, &latest_currents.mv_a);
//     adc_raw_to_millivolts_dt(&adc_sob, &latest_currents.mv_b);
//     adc_raw_to_millivolts_dt(&adc_soc, &latest_currents.mv_c);

//     /* Output copy back to thread target structure */
//     *out_samples = latest_currents;

//     return 0;
// }

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

// Shell command to manually trigger the process (for testing)
//  static int shell_read_currents(const struct shell *sh, size_t argc, char **argv)
//  {
//      shell_print(sh, "Current values:");
//      struct motor_current_samples csa_adc_samples;

// 	adc_manager_read_currents(&csa_adc_samples);
//     shell_print(sh, "Currents Raw -> A: %d, B: %d, C: %d | Voltage -> A: %d mV, B: %d mV, C: %d mV",
//                 csa_adc_samples.raw_a, csa_adc_samples.raw_b, csa_adc_samples.raw_c,
//                 csa_adc_samples.mv_a, csa_adc_samples.mv_b, csa_adc_samples.mv_c);

//     return 0;
// }


/* Custom ISR wrapper to safely bridge HAL and Zephyr */
static void adc_foc_isr(const void *arg)
{
    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)arg;
    
    /* Check and clear Injected End of Sequence flag */
    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS)) {
        __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOS);
        
        /* Fetch injected conversion results */
        uint32_t i_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint32_t i_b = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        uint32_t i_c = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);

        /* Process FOC calculations or signal a thread here */
    }
}

void init_adcs_simultaneous_foc(void)
{
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    hadc1.Instance = ADC1;
    hadc2.Instance = ADC2;
    hadc3.Instance = ADC3;

    /* Base configuration for ADC1 (Master) */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.QueueInjectedContext = DISABLE;

    /* Hardware Trigger from TIM1 TRGO */
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;

    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    /* Configuration for ADC2 */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    /* Configuration for ADC3 */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
    HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

    /* Enable Interrupt */
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

    /* Connect native ISR wrapper */
    IRQ_CONNECT(ADC_IRQn, 0, adc_foc_isr, &hadc1, 0);
    irq_enable(ADC_IRQn);
}

/* Register the command "start_process" */
// SHELL_CMD_REGISTER(read_currents, NULL, "Read current values", shell_read_currents);
