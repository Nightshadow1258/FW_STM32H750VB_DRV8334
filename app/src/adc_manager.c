/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_manager.h"

#define ADC_VREF_MV    3300
#define ADC_FULL_SCALE 65535  /* 16 bit resolution for STM32H7 ADC */

static struct motor_current_samples latest_currents;
static bool adc_ready = false;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

static inline int32_t raw_to_mv(int16_t raw)
{
    return ((int32_t)raw * ADC_VREF_MV) / ADC_FULL_SCALE;
}

static void adc_foc_isr(const void *arg)
{
    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)arg;
    
    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS)) {
        __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOS);
        
        latest_currents.raw_a = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        latest_currents.raw_b = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        latest_currents.raw_c = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);

        latest_currents.mv_a = raw_to_mv(latest_currents.raw_a);
        latest_currents.mv_b = raw_to_mv(latest_currents.raw_b);
        latest_currents.mv_c = raw_to_mv(latest_currents.raw_c);
    }
}


void get_latest_currents(struct motor_current_samples *samples)
{
    if (samples != NULL) {
        *samples = latest_currents;
    }
}

bool is_adc_ready(void)
{
    return adc_ready;
}

static int shell_read_currents(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct motor_current_samples samples;
    get_latest_currents(&samples);

    shell_print(sh, "Phase A: RAW = %d, Voltage = %d mV", samples.raw_a, samples.mv_a);
    shell_print(sh, "Phase B: RAW = %d, Voltage = %d mV", samples.raw_b, samples.mv_b);
    shell_print(sh, "Phase C: RAW = %d, Voltage = %d mV", samples.raw_c, samples.mv_c);

    return 0;
}

SHELL_CMD_REGISTER(read_currents, NULL, "Read current values in raw and mV", shell_read_currents);

void print_adc_channel_debug(uint8_t adc_num, uint32_t channel)
{
    ADC_TypeDef *adc = NULL;

    switch (adc_num) {
    case 1:
        adc = ADC1;
        break;
    case 2:
        adc = ADC2;
        break;
    case 3:
        adc = ADC3;
        break;
    default:
        printk("Invalid ADC instance: %d (Select 1, 2, or 3)\n", adc_num);
        return;
    }

    printk("\n======================================================\n");
    printk("       FULL DEBUG REPORT FOR ADC%d (Channel %u)\n", adc_num, (unsigned int)channel);
    printk("======================================================\n");

    /* --- GLOBAL ADC COMMON REGISTERS --- */
    printk("\n[COMMON REGISTERS]\n");
    printk("  ADC12_CCR  : 0x%08X (DUALMODE=0x%X, PRESC=0x%X, CKMODE=0x%X)\n",
           (unsigned int)ADC12_COMMON->CCR,
           (unsigned int)((ADC12_COMMON->CCR & ADC_CCR_DUAL_Msk) >> ADC_CCR_DUAL_Pos),
           (unsigned int)((ADC12_COMMON->CCR & ADC_CCR_PRESC_Msk) >> ADC_CCR_PRESC_Pos),
           (unsigned int)((ADC12_COMMON->CCR & ADC_CCR_CKMODE_Msk) >> ADC_CCR_CKMODE_Pos));

    /* --- ADC CONTROL & STATUS REGISTERS --- */
    printk("\n[CONTROL & STATUS]\n");
    printk("  ISR (Status)      : 0x%08X\n", (unsigned int)adc->ISR);
    printk("    -> ADRDY  (Bit 0)  : %u\n", (adc->ISR & ADC_ISR_ADRDY) ? 1 : 0);
    printk("    -> EOSMP  (Bit 1)  : %u\n", (adc->ISR & ADC_ISR_EOSMP) ? 1 : 0);
    printk("    -> EOC    (Bit 2)  : %u\n", (adc->ISR & ADC_ISR_EOC) ? 1 : 0);
    printk("    -> JEOC   (Bit 5)  : %u\n", (adc->ISR & ADC_ISR_JEOC) ? 1 : 0);
    printk("    -> JEOS   (Bit 6)  : %u\n", (adc->ISR & ADC_ISR_JEOS) ? 1 : 0);
    printk("  IER (Interrupt En): 0x%08X\n", (unsigned int)adc->IER);
    printk("  CR  (Control)     : 0x%08X\n", (unsigned int)adc->CR);
    printk("    -> ADEN     (Bit 0) : %u\n", (adc->CR & ADC_CR_ADEN) ? 1 : 0);
    printk("    -> ADSTART  (Bit 2) : %u\n", (adc->CR & ADC_CR_ADSTART) ? 1 : 0);
    printk("    -> JADSTART (Bit 3) : %u\n", (adc->CR & ADC_CR_JADSTART) ? 1 : 0);
    printk("    -> ADVREGEN (Bit 28): %u\n", (adc->CR & ADC_CR_ADVREGEN) ? 1 : 0);
    printk("    -> DEEPPWD  (Bit 29): %u\n", (adc->CR & ADC_CR_DEEPPWD) ? 1 : 0);
    printk("    -> ADCAL    (Bit 31): %u\n", (adc->CR & ADC_CR_ADCAL) ? 1 : 0);

    /* --- CONFIGURATION REGISTERS --- */
    printk("\n[CONFIGURATION]\n");
    printk("  CFGR  : 0x%08X (RES=0x%X, EXTEN=0x%X, EXTSEL=0x%X, CONT=%u)\n",
           (unsigned int)adc->CFGR,
           (unsigned int)((adc->CFGR & ADC_CFGR_RES_Msk) >> ADC_CFGR_RES_Pos),
           (unsigned int)((adc->CFGR & ADC_CFGR_EXTEN_Msk) >> ADC_CFGR_EXTEN_Pos),
           (unsigned int)((adc->CFGR & ADC_CFGR_EXTSEL_Msk) >> ADC_CFGR_EXTSEL_Pos),
           (adc->CFGR & ADC_CFGR_CONT) ? 1 : 0);
    printk("  CFGR2 : 0x%08X (ROVSE=%u, JOVSE=%u, OVSR=0x%X, OVSS=0x%X)\n",
           (unsigned int)adc->CFGR2,
           (adc->CFGR2 & ADC_CFGR2_ROVSE) ? 1 : 0,
           (adc->CFGR2 & ADC_CFGR2_JOVSE) ? 1 : 0,
           (unsigned int)((adc->CFGR2 & ADC_CFGR2_OVSR_Msk) >> ADC_CFGR2_OVSR_Pos),
           (unsigned int)((adc->CFGR2 & ADC_CFGR2_OVSS_Msk) >> ADC_CFGR2_OVSS_Pos));

    /* --- INJECTED SEQUENCE REGISTERS --- */
    printk("\n[INJECTED SEQUENCE (JSQR)]\n");
    printk("  JSQR  : 0x%08X\n", (unsigned int)adc->JSQR);
    printk("    -> JL      (Len) : %u channel(s)\n", (unsigned int)(((adc->JSQR & ADC_JSQR_JL_Msk) >> ADC_JSQR_JL_Pos) + 1));
    printk("    -> JEXTEN  (Edge): 0x%X\n", (unsigned int)((adc->JSQR & ADC_JSQR_JEXTEN_Msk) >> ADC_JSQR_JEXTEN_Pos));
    printk("    -> JEXTSEL (Trig): 0x%X\n", (unsigned int)((adc->JSQR & ADC_JSQR_JEXTSEL_Msk) >> ADC_JSQR_JEXTSEL_Pos));
    printk("    -> JSQ1    (Ch)  : %u\n", (unsigned int)((adc->JSQR & ADC_JSQR_JSQ1_Msk) >> ADC_JSQR_JSQ1_Pos));

    /* --- REGULAR SEQUENCE REGISTERS --- */
    printk("\n[REGULAR SEQUENCE (SQR1-SQR4)]\n");
    printk("  SQR1  : 0x%08X (L=%u, SQ1=%u, SQ2=%u)\n",
           (unsigned int)adc->SQR1,
           (unsigned int)((adc->SQR1 & ADC_SQR1_L_Msk) >> ADC_SQR1_L_Pos),
           (unsigned int)((adc->SQR1 & ADC_SQR1_SQ1_Msk) >> ADC_SQR1_SQ1_Pos),
           (unsigned int)((adc->SQR1 & ADC_SQR1_SQ2_Msk) >> ADC_SQR1_SQ2_Pos));
    printk("  SQR2  : 0x%08X\n", (unsigned int)adc->SQR2);

    /* --- CHANNEL SAMPLING TIME --- */
    printk("\n[SAMPLE TIME REGISTERS]\n");
    printk("  SMPR1 : 0x%08X\n", (unsigned int)adc->SMPR1);
    printk("  SMPR2 : 0x%08X\n", (unsigned int)adc->SMPR2);

    /* Calculate specific sample time for requested channel */
    uint32_t smpr_val = 0;
    if (channel <= 9) {
        smpr_val = (adc->SMPR1 >> (channel * 3)) & 0x7;
    } else if (channel <= 19) {
        smpr_val = (adc->SMPR2 >> ((channel - 10) * 3)) & 0x7;
    }
    printk("  -> Configured Sample Time for Channel %u : 0x%X\n", (unsigned int)channel, (unsigned int)smpr_val);

    /* --- DATA & OFFSETS --- */
    printk("\n[DATA & OFFSET REGISTERS]\n");
    printk("  DR   (Regular Data)  : %u (0x%04X)\n", (unsigned int)adc->DR, (unsigned int)adc->DR);
    printk("  JDR1 (Injected Data 1): %u (0x%04X)\n", (unsigned int)adc->JDR1, (unsigned int)adc->JDR1);
    
    printk("  OFR1 (Offset 1 Reg)   : 0x%08X (OFFSET1_CH=%u, OFFSET1=%u, ENABLED=%u)\n",
           (unsigned int)adc->OFR1,
           (unsigned int)((adc->OFR1 & ADC_OFR1_OFFSET1_CH_Msk) >> ADC_OFR1_OFFSET1_CH_Pos),
           (unsigned int)((adc->OFR1 & ADC_OFR1_OFFSET1_Msk) >> ADC_OFR1_OFFSET1_Pos),
           (adc->OFR1 & (1U << 31)) ? 1 : 0);

    printk("  PCSEL (Pin Select)    : 0x%08X (Ch %u selected: %u)\n",
           (unsigned int)adc->PCSEL,
           (unsigned int)channel,
           (adc->PCSEL & (1U << channel)) ? 1 : 0);

    printk("======================================================\n\n");
}

static int cmd_dump_adc_channel(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_error(sh, "Usage: dump_adc <adc_num 1-3> <channel_num 0-19>");
        return -EINVAL;
    }

    uint8_t adc_num = (uint8_t)strtoul(argv[1], NULL, 10);
    uint32_t channel = (uint32_t)strtoul(argv[2], NULL, 10);

    print_adc_channel_debug(adc_num, channel);

    return 0;
}

SHELL_CMD_REGISTER(dump_adc, NULL, "Dump hardware registers for an ADC and channel", cmd_dump_adc_channel);