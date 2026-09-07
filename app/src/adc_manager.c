/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_manager.h"

#define ADC_VREF_MV 3300
#define ADC_FULL_SCALE 65535 /* 16 bit resolution for STM32H7 ADC */

static volatile struct motor_current_sample latest_currents;
volatile uint32_t g_adc_isr_count = 0;
PINCTRL_DT_DEFINE(DT_NODELABEL(adc1));
static ADC_HandleTypeDef hadc1;

static inline uint32_t raw_to_mv(uint16_t raw)
{
    return ((uint32_t)raw * ADC_VREF_MV) / ADC_FULL_SCALE;
}

int manual_adc1_init(void)
{
    /* 1. Apply Pin Control from DTS */
    const struct pinctrl_dev_config *pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(adc1));
    int ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0)
    {
        return ret;
    }

    /* 2. Enable ADC Clock */
    __HAL_RCC_ADC12_CLK_ENABLE();

    uint32_t difsel = ADC1->DIFSEL;
    if (difsel & (1U << 9))
    {
        /* Clear bit 9 to force Single-Ended mode */
        ADC1->DIFSEL &= ~(1U << 9);
    }

    /* 3. Configure ADC Instance */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;

    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        return -1;
    }

    hadc1.Instance->CR &= ~ADC_CR_DEEPPWD; /* Exit Deep Power Down */
    hadc1.Instance->CR |= ADC_CR_ADVREGEN; /* Enable ADC Internal Regulator */

    /* Wait for LDO startup time (20us required by STM32H7 datasheet) */
    k_busy_wait(20);

    /* 4. Voltage Regulator and Calibration */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return -2;
    }

    /* 5. Channel Configuration */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5; /* Max sampling time to eliminate impedance offset */
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return -3;
    }
    /* Start ADC in Interrupt or DMA mode to receive conversions on TIM1 trigger events */
    return 0;

}

static void adc_isr_handler(const void *arg)
{
    ARG_UNUSED(arg);
    HAL_ADC_IRQHandler(&hadc1);
}

void setup_adc_interrupt(void)
{
/* Connect ADC IRQ line to Zephyr kernel vector table */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc1)),
                0,                  /* Direct priority value (0-15 on ARM Cortex-M) */
                adc_isr_handler,
                NULL,
                0);                 /* Flags */

    /* Enable interrupt line in NVIC */
    irq_enable(DT_IRQN(DT_NODELABEL(adc1)));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        /* Read converted value from DR register (clears EOC flag automatically) */
        latest_currents.raw_a = (uint16_t)HAL_ADC_GetValue(hadc);
        g_adc_isr_count++;
    }
}

int start_timer_triggered_adc_isr(void)
{
    int ret = manual_adc1_init();
    if (ret < 0) {
        return ret;
    }
    
    /* 1. Register and enable NVIC interrupt */
    setup_adc_interrupt();

    /* 2. Start ADC in Interrupt Mode (waits for TIM1 TRGO signal) */
    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        return -1;
    }

    return 0;
}

static int cmd_read_isr_val(const struct shell *sh, size_t argc, char **argv)
{
ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint16_t raw = latest_currents.raw_a;
    uint32_t count = g_adc_isr_count;
    uint32_t mv = raw_to_mv(raw);

    shell_print(sh, "ISR Conversion Count : %u", count);
    shell_print(sh, "Latest Raw ADC Value : %u (0x%04X)", raw, raw);
    shell_print(sh, "Calculated Voltage   : %u mV", mv);

    return 0;
}

SHELL_CMD_REGISTER(read_isr, NULL, "Read current ISR updated ADC value", cmd_read_isr_val);


uint32_t manual_adc1_read_single(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        return HAL_ADC_GetValue(&hadc1);
    }
    return 0;
}

static int cmd_adc_init(const struct shell *sh, size_t argc, char **argv)
{
    int ret = manual_adc1_init();
    if (ret < 0)
    {
        shell_error(sh, "ADC initialization failed: %d", ret);
        return ret;
    }

    shell_print(sh, "ADC initialized successfully.");
    return 0;
}

static int cmd_adc_read(const struct shell *sh, size_t argc, char **argv)
{

    latest_currents.raw_a = (uint16_t)manual_adc1_read_single();
    latest_currents.mv_a = raw_to_mv((uint16_t)latest_currents.raw_a);
    shell_print(sh, "ADC Raw Value: %u", latest_currents.raw_a);
    shell_print(sh, "ADC Millivolts: %u mV", latest_currents.mv_a);
    return 0;
}

/* Register parent command 'custom_adc' with subcommands 'init' and 'read' */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_adc,
                               SHELL_CMD(init, NULL, "Initialize ADC1 manually", cmd_adc_init),
                               SHELL_CMD(read, NULL, "Read single value from ADC1", cmd_adc_read),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(custom_adc, &sub_adc, "Custom STM32 ADC commands", NULL);

static int cmd_dump_adc_channel(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3)
    {
        shell_error(sh, "Usage: dump_adc <adc_num 1-3> <channel_num 0-19>");
        return -EINVAL;
    }

    uint8_t adc_num = (uint8_t)strtoul(argv[1], NULL, 10);
    uint32_t channel = (uint32_t)strtoul(argv[2], NULL, 10);
    ADC_TypeDef *adc = NULL;

    switch (adc_num)
    {
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
        shell_error(sh, "Invalid ADC instance: %d", adc_num);
        return -EINVAL;
    }

    shell_print(sh, "======================================================");
    shell_print(sh, "       FULL DEBUG REPORT FOR ADC%d (Channel %u)", adc_num, (unsigned int)channel);
    shell_print(sh, "======================================================");

    shell_print(sh, "\n[COMMON REGISTERS]");
    shell_print(sh, "  ADC12_CCR : 0x%08X", (unsigned int)ADC12_COMMON->CCR);
    shell_print(sh, "  ADC3_CCR : 0x%08X", (unsigned int)ADC3_COMMON->CCR);

    shell_print(sh, "\n[CONTROL & STATUS]");
    shell_print(sh, "  ISR       : 0x%08X", (unsigned int)adc->ISR);
    shell_print(sh, "  IER       : 0x%08X", (unsigned int)adc->IER);
    shell_print(sh, "  CR        : 0x%08X", (unsigned int)adc->CR);

    shell_print(sh, "\n[CONFIGURATION]");
    shell_print(sh, "  CFGR      : 0x%08X", (unsigned int)adc->CFGR);
    shell_print(sh, "  CFGR2     : 0x%08X", (unsigned int)adc->CFGR2);

    shell_print(sh, "\n[SAMPLING TIME REGISTERS]");
    shell_print(sh, "  SMPR1     : 0x%08X", (unsigned int)adc->SMPR1);
    shell_print(sh, "  SMPR2     : 0x%08X", (unsigned int)adc->SMPR2);

    shell_print(sh, "\n[WATCHDOG THRESHOLD REGISTERS]");
    shell_print(sh, "  LTR1      : 0x%08X", (unsigned int)adc->LTR1);
    shell_print(sh, "  HTR1      : 0x%08X", (unsigned int)adc->HTR1);
    shell_print(sh, "  LTR2      : 0x%08X", (unsigned int)adc->LTR2);
    shell_print(sh, "  HTR2      : 0x%08X", (unsigned int)adc->HTR2);
    shell_print(sh, "  LTR3      : 0x%08X", (unsigned int)adc->LTR3);
    shell_print(sh, "  HTR3      : 0x%08X", (unsigned int)adc->HTR3);

    shell_print(sh, "\n[REGULAR SEQUENCE REGISTERS]");
    shell_print(sh, "  SQR1      : 0x%08X", (unsigned int)adc->SQR1);
    shell_print(sh, "  SQR2      : 0x%08X", (unsigned int)adc->SQR2);
    shell_print(sh, "  SQR3      : 0x%08X", (unsigned int)adc->SQR3);
    shell_print(sh, "  SQR4      : 0x%08X", (unsigned int)adc->SQR4);

    shell_print(sh, "\n[INJECTED SEQUENCE REGISTERS]");
    shell_print(sh, "  JSQR      : 0x%08X", (unsigned int)adc->JSQR);

    shell_print(sh, "\n[OFFSET REGISTERS]");
    shell_print(sh, "  OFR1      : 0x%08X", (unsigned int)adc->OFR1);
    shell_print(sh, "  OFR2      : 0x%08X", (unsigned int)adc->OFR2);
    shell_print(sh, "  OFR3      : 0x%08X", (unsigned int)adc->OFR3);
    shell_print(sh, "  OFR4      : 0x%08X", (unsigned int)adc->OFR4);

    shell_print(sh, "\n[DATA REGISTERS]");
    shell_print(sh, "  DR        : %u (0x%04X)", (unsigned int)adc->DR, (unsigned int)adc->DR);
    shell_print(sh, "  JDR1      : %u (0x%04X)", (unsigned int)adc->JDR1, (unsigned int)adc->JDR1);
    shell_print(sh, "  JDR2      : %u (0x%04X)", (unsigned int)adc->JDR2, (unsigned int)adc->JDR2);
    shell_print(sh, "  JDR3      : %u (0x%04X)", (unsigned int)adc->JDR3, (unsigned int)adc->JDR3);
    shell_print(sh, "  JDR4      : %u (0x%04X)", (unsigned int)adc->JDR4, (unsigned int)adc->JDR4);

    shell_print(sh, "\n[DIFFERENTIAL & CALIBRATION]");
    shell_print(sh, "  DIFSEL    : 0x%08X", (unsigned int)adc->DIFSEL);
    shell_print(sh, "  CALFACT   : 0x%08X", (unsigned int)adc->CALFACT);
    shell_print(sh, "  CALFACT2  : 0x%08X", (unsigned int)adc->CALFACT2);
    shell_print(sh, "======================================================\n");

    return 0;
}

SHELL_CMD_REGISTER(dump_adc, NULL, "Dump hardware registers for an ADC and channel", cmd_dump_adc_channel);

static int shell_check_adc_freq(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    /* 1. Get Clock Source selection from RCC */
    uint32_t d3ccipr = RCC->D3CCIPR;
    uint32_t adc_sel = (d3ccipr & RCC_D3CCIPR_ADCSEL) >> RCC_D3CCIPR_ADCSEL_Pos;

    const char *source_name = "UNKNOWN";
    uint32_t f_source = 0;

    switch (adc_sel)
    {
    case 0:
        source_name = "PLL2_P";
        {
            /* Read HSE frequency from devicetree (e.g. 16 MHz) */
            uint32_t hse_freq = DT_PROP(DT_NODELABEL(clk_hse), clock_frequency);

            /* Read PLL2 Dividers from hardware */
            uint32_t divm2 = (RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM2) >> RCC_PLLCKSELR_DIVM2_Pos;
            uint32_t divn2 = ((RCC->PLL2DIVR & RCC_PLL2DIVR_N2) >> RCC_PLL2DIVR_N2_Pos) + 1;
            uint32_t divp2 = ((RCC->PLL2DIVR & RCC_PLL2DIVR_P2) >> RCC_PLL2DIVR_P2_Pos) + 1;

            if (divm2 > 0)
            {
                f_source = ((hse_freq / divm2) * divn2) / divp2;
            }
        }
        break;

    case 1:
        source_name = "PLL3_R";
        {
            uint32_t hse_freq = DT_PROP(DT_NODELABEL(clk_hse), clock_frequency);
            uint32_t divm3 = (RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM3) >> RCC_PLLCKSELR_DIVM3_Pos;
            uint32_t divn3 = ((RCC->PLL3DIVR & RCC_PLL3DIVR_N3) >> RCC_PLL3DIVR_N3_Pos) + 1;
            uint32_t divr3 = ((RCC->PLL3DIVR & RCC_PLL3DIVR_R3) >> RCC_PLL3DIVR_R3_Pos) + 1;

            if (divm3 > 0)
            {
                f_source = ((hse_freq / divm3) * divn3) / divr3;
            }
        }
        break;

    case 2:
        source_name = "PER_CLK (HSI/HSE/CSI)";
        f_source = 16000000U; /* Default HSI */
        break;

    default:
        break;
    }

    /* 2. Read ADC prescaler from ADC12_COMMON */
    uint32_t ccr = ADC12_COMMON->CCR;
    uint32_t presc_val = (ccr & ADC_CCR_PRESC) >> ADC_CCR_PRESC_Pos;

    static const uint8_t div_map[] = {1, 2, 4, 6, 8, 10, 12, 16, 32};
    uint32_t divider = (presc_val < ARRAY_SIZE(div_map)) ? div_map[presc_val] : 1;

    /* 3. Compute final f_ADC */
    uint32_t f_adc = f_source / divider;

    /* Read PLL2 Configuration Registers */
    uint32_t pllckselr = RCC->PLLCKSELR;
    uint32_t pll2divr = RCC->PLL2DIVR;
    uint32_t pll2fracr = RCC->PLL2FRACR;

    uint32_t divm2 = (pllckselr & RCC_PLLCKSELR_DIVM2) >> RCC_PLLCKSELR_DIVM2_Pos;
    uint32_t divn2 = ((pll2divr & RCC_PLL2DIVR_N2) >> RCC_PLL2DIVR_N2_Pos) + 1;
    uint32_t divp2 = ((pll2divr & RCC_PLL2DIVR_P2) >> RCC_PLL2DIVR_P2_Pos) + 1;
    uint32_t fracn2 = (pll2fracr & RCC_PLL2FRACR_FRACN2) >> RCC_PLL2FRACR_FRACN2_Pos;

    shell_print(sh, "--- PLL2 RAW REGISTER DUMP ---");
    shell_print(sh, "DIVM2  : %u", divm2);
    shell_print(sh, "DIVN2  : %u", divn2);
    shell_print(sh, "DIVP2  : %u", divp2);
    shell_print(sh, "FRACN2 : %u", fracn2);

    shell_print(sh, "--- ADC CLOCK DIAGNOSTICS ---");
    shell_print(sh, "ADC Clock Source : %s (ADCSEL = %u)", source_name, adc_sel);
    shell_print(sh, "Raw Source Freq  : %u Hz (%u MHz)", f_source, f_source / 1000000U);
    shell_print(sh, "ADC Prescaler    : Div %u (PRESC = 0x%X)", divider, presc_val);
    shell_print(sh, "Actual f_ADC     : %u Hz (%u.%02u MHz)",
                f_adc, f_adc / 1000000U, (f_adc % 1000000U) / 10000U);

    if (f_adc > 50000000U)
    {
        shell_error(sh, "WARNING: f_ADC exceeds maximum hardware limit (50 MHz)!");
    }

    return 0;
}

SHELL_CMD_REGISTER(check_adc_freq, NULL, "Check ADC clock frequency", shell_check_adc_freq);

static int cmd_check_pb0_gpio(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint32_t moder = GPIOB->MODER;
    uint32_t pupdr = GPIOB->PUPDR;

    /* PB0 uses bits [1:0] in both MODER and PUPDR */
    uint32_t pb0_mode = (moder >> (0 * 2)) & 0x03U;
    uint32_t pb0_pupd = (pupdr >> (0 * 2)) & 0x03U;

    shell_print(sh, "==========================================");
    shell_print(sh, "       GPIOB PIN 0 REGISTER STATUS        ");
    shell_print(sh, "==========================================");
    shell_print(sh, "GPIOB->MODER : 0x%08X (PB0 Mode Bits: 0b%u%u)",
                (unsigned int)moder,
                (unsigned int)((pb0_mode >> 1) & 0x01),
                (unsigned int)(pb0_mode & 0x01));

    shell_print(sh, "GPIOB->PUPDR : 0x%08X (PB0 Pull Bits: 0b%u%u)",
                (unsigned int)pupdr,
                (unsigned int)((pb0_pupd >> 1) & 0x01),
                (unsigned int)(pb0_pupd & 0x01));

    shell_print(sh, "------------------------------------------");

    /* Evaluate MODER [1:0] */
    if (pb0_mode == 0x03U)
    {
        shell_print(sh, "MODE : [OK] Analog Mode (11)");
    }
    else
    {
        shell_error(sh, "MODE : [FAIL] Not in Analog Mode! (00=Input, 01=Output, 10=AF)");
    }

    /* Evaluate PUPDR [1:0] */
    if (pb0_pupd == 0x00U)
    {
        shell_print(sh, "PULL : [OK] No Pull-Up / No Pull-Down (00)");
    }
    else if (pb0_pupd == 0x01U)
    {
        shell_error(sh, "PULL : [FAIL] Internal Pull-Up Active! (01)");
    }
    else if (pb0_pupd == 0x02U)
    {
        shell_error(sh, "PULL : [FAIL] Internal Pull-Down Active! (10)");
    }

    shell_print(sh, "==========================================\n");

    return 0;
}

SHELL_CMD_REGISTER(check_pb0, NULL, "Verify GPIOB Pin 0 analog mode configuration", cmd_check_pb0_gpio);

static int cmd_adc_read_avg(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint32_t sum = 0;
    const uint32_t samples = 16;

    for (uint32_t i = 0; i < samples; i++)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(&hadc1);
        }
        else
        {
            shell_error(sh, "ADC Conversion Timeout at sample %u", i);
            return -EIO;
        }
        k_busy_wait(10);
    }

    uint16_t avg_raw = (uint16_t)(sum / samples);
    uint32_t mv = ((uint32_t)avg_raw * ADC_VREF_MV) / ADC_FULL_SCALE;

    /* Raw Register Bit Inspection */
    uint32_t cfgr = ADC1->CFGR;
    uint32_t dr = ADC1->DR;

    shell_print(sh, "==========================================");
    shell_print(sh, "       ADC1 READ REPORT (16x AVG)         ");
    shell_print(sh, "==========================================");
    shell_print(sh, "ADC1->CFGR Register : 0x%08X", (unsigned int)cfgr);
    shell_print(sh, "Last ADC1->DR Read  : %u (0x%04X)", (unsigned int)dr, (unsigned int)dr);
    shell_print(sh, "16x Averaged Raw    : %u (0x%04X)", avg_raw, avg_raw);
    shell_print(sh, "Calculated Voltage  : %u mV", mv);
    shell_print(sh, "==========================================\n");

    return 0;
}

SHELL_CMD_REGISTER(read_pb0_avg, NULL, "Read PB0 (ADC1_INP9) with 16x averaging", cmd_adc_read_avg);

static int cmd_verify_adc1_ch9_routing(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint32_t moder = GPIOB->MODER;
    uint32_t pupdr = GPIOB->PUPDR;
    uint32_t sqr1 = ADC1->SQR1;

    uint32_t pb0_mode = (moder >> 0) & 0x03U;
    uint32_t pb0_pupd = (pupdr >> 0) & 0x03U;
    uint32_t sq1_chan = (sqr1 >> 6) & 0x1FU; /* SQ1 bits [10:6] */

    shell_print(sh, "==========================================");
    shell_print(sh, "     ADC1 CH9 / PB0 ROUTING DIAGNOSTIC    ");
    shell_print(sh, "==========================================");
    shell_print(sh, "GPIOB->MODER[1:0] : 0b%u%u (Expected 0b11)", (pb0_mode >> 1) & 1, pb0_mode & 1);
    shell_print(sh, "GPIOB->PUPDR[1:0] : 0b%u%u (Expected 0b00)", (pb0_pupd >> 1) & 1, pb0_pupd & 1);
    shell_print(sh, "ADC1->SQR1 SQ1    : %u (Expected 9)", sq1_chan);
    shell_print(sh, "------------------------------------------");

    bool ready = true;
    if (pb0_mode != 0x03U)
    {
        shell_error(sh, "FAIL: PB0 is not in Analog Mode.");
        ready = false;
    }
    if (pb0_pupd != 0x00U)
    {
        shell_error(sh, "FAIL: Internal pull-up or pull-down active on PB0.");
        ready = false;
    }
    if (sq1_chan != 9)
    {
        shell_error(sh, "FAIL: ADC1 is configured for Channel %u, not Channel 9.", sq1_chan);
        ready = false;
    }

    if (ready)
    {
        shell_print(sh, "SUCCESS: Hardware and ADC sequence correctly map PB0 -> ADC1 Channel 9.");
    }
    shell_print(sh, "==========================================\n");

    return 0;
}

SHELL_CMD_REGISTER(check_adc_routing, NULL, "Verify PB0 to ADC1 CH9 routing registers", cmd_verify_adc1_ch9_routing);