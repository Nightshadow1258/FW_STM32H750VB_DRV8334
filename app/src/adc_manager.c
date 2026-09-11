/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_manager.h"

#define ADC_VREF_MV 3300
#define ADC_FULL_SCALE 65535 /* 16 bit resolution for STM32H7 ADC */
#define ADC_BUFFER_SIZE 1

/* 32-bit aligned array buffer to store 8 hardware measurements */
static uint16_t adc1_dma_buffer[ADC_BUFFER_SIZE] __attribute__((aligned(32)));
static uint16_t adc2_dma_buffer[ADC_BUFFER_SIZE] __attribute__((aligned(32)));
static uint16_t adc3_dma_buffer[ADC_BUFFER_SIZE] __attribute__((aligned(32)));

volatile struct motor_currents_sample g_latest_currents = {0};
volatile uint32_t g_adc_isr_count = 0;

#define OFFSET_RAW_MIN 22837U /* ~1150 mV */
#define OFFSET_RAW_MAX 42698U /* ~2150 mV */
#define CALIBRATION_SAMPLES 64U

static bool g_offsets_calibrated = false;

PINCTRL_DT_DEFINE(DT_NODELABEL(adc1));
static ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;

PINCTRL_DT_DEFINE(DT_NODELABEL(adc2));
static ADC_HandleTypeDef hadc2;
static DMA_HandleTypeDef hdma_adc2;

PINCTRL_DT_DEFINE(DT_NODELABEL(adc3));
static ADC_HandleTypeDef hadc3;
static DMA_HandleTypeDef hdma_adc3;

static inline uint32_t raw_to_mv(uint16_t raw)
{
    return ((uint32_t)raw * ADC_VREF_MV) / ADC_FULL_SCALE;
}
int manual_adc_dma_init(DMA_HandleTypeDef *hdma,
                        ADC_HandleTypeDef *hadc,
                        DMA_Stream_TypeDef *stream_instance,
                        uint32_t request)
{
    /* 1. Enable DMA Clocks */
    if (stream_instance >= DMA1_Stream0 && stream_instance <= DMA1_Stream7)
    {
        __HAL_RCC_DMA1_CLK_ENABLE();
    }
    else if (stream_instance >= DMA2_Stream0 && stream_instance <= DMA2_Stream7)
    {
        __HAL_RCC_DMA2_CLK_ENABLE();
    }

    /* 2. Configure DMA Stream */
    hdma->Instance = stream_instance;
    hdma->Init.Request = request;
    hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma->Init.MemInc = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma->Init.Mode = DMA_CIRCULAR;
    hdma->Init.Priority = DMA_PRIORITY_HIGH;
    hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(hdma) != HAL_OK)
    {
        return -1;
    }

    /* Link DMA handle to ADC handle */
    __HAL_LINKDMA(hadc, DMA_Handle, *hdma);

    return 0;
}

int manual_adc_init(ADC_HandleTypeDef *hadc,
                    ADC_TypeDef *instance,
                    const struct pinctrl_dev_config *pcfg,
                    uint32_t channel)
{
    int ret;

    /* 1. Apply Pin Control from DTS */
    if (pcfg != NULL)
    {
        ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
        if (ret < 0)
        {
            return ret;
        }
    }

    /* 2. Enable ADC Clock */
    if (instance == ADC1 || instance == ADC2)
    {
        __HAL_RCC_ADC12_CLK_ENABLE();
    }
#ifdef ADC3
    else if (instance == ADC3)
    {
        __HAL_RCC_ADC3_CLK_ENABLE();
    }
#endif

    /* Force Single Ended mode on selected channel */
    if (channel < 32)
    {
        instance->DIFSEL &= ~(1U << channel);
    }

    /* 3. Configure ADC Instance */
    hadc->Instance = instance;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_16B;
    hadc->Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 1;
    hadc->Init.DiscontinuousConvMode = DISABLE;

    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

    hadc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
        return -1;
    }

    hadc->Instance->CR &= ~ADC_CR_DEEPPWD; /* Exit Deep Power Down */
    hadc->Instance->CR |= ADC_CR_ADVREGEN; /* Enable ADC Internal Regulator */

    /* Wait for LDO startup time */
    k_busy_wait(20);

    /* 4. Voltage Regulator and Calibration */
    if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return -2;
    }

    /* 5. Channel Configuration */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        return -3;
    }

    return 0;
}

int start_timer_triggered_adc_dma(void)
{
    int ret = -1;
    /* 1. Complete ADC1 hardware configuration */
    const struct pinctrl_dev_config *pcfg1 = PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(adc1));
    ret = manual_adc_init(&hadc1, ADC1, pcfg1, ADC_CHANNEL_9); /* Example: Channel 9 for Phase A current */
    if (ret < 0)
    {
        return ret;
    }
    const struct pinctrl_dev_config *pcfg2 = PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(adc2));
    ret = manual_adc_init(&hadc2, ADC2, pcfg2, ADC_CHANNEL_5); /* Example: Channel 5 for Phase B current */
    if (ret < 0)
    {
        return ret;
    }

    const struct pinctrl_dev_config *pcfg3 = PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(adc3));
    ret = manual_adc_init(&hadc3, ADC3, pcfg3, ADC_CHANNEL_1); /* Example: Channel 1 for Phase C current */
    if (ret < 0)
    {
        return ret;
    }

    /* 2. Initialize DMA2 Stream 0 */
    ret = manual_adc_dma_init(&hdma_adc1, &hadc1, DMA2_Stream0, DMA_REQUEST_ADC1);
    if (ret < 0)
    {
        return ret;
    }

    /* 2. Initialize DMA2 Stream 0 */
    ret = manual_adc_dma_init(&hdma_adc2, &hadc2, DMA2_Stream1, DMA_REQUEST_ADC2);
    if (ret < 0)
    {
        return ret;
    }

    /* 2. Initialize DMA2 Stream 0 */
    ret = manual_adc_dma_init(&hdma_adc3, &hadc3, DMA2_Stream2, DMA_REQUEST_ADC3);
    if (ret < 0)
    {
        return ret;
    }

    /* 3. Disable active CPU interrupts on ADC line */
    irq_disable(DT_IRQN(DT_NODELABEL(adc1)));
    irq_disable(DT_IRQN(DT_NODELABEL(adc2)));
    irq_disable(DT_IRQN(DT_NODELABEL(adc3)));

    /* 4. Start ADC in Circular DMA mode with 8-sample buffer size */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
        return -4;
    }

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_dma_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
        return -4;
    }

    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_dma_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
        return -4;
    }

    return 0;
}

int calibrate_current_offsets(void)
{
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;
    uint32_t sum_c = 0;

    /* Ensure DMA circular sampling is active */
    if ((hadc1.Instance->CR & ADC_CR_ADSTART) == 0)
    {
        return -EAGAIN;
    }

    for (uint32_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        /* Wait for next DMA write / Timer trigger cycle */
        k_busy_wait(50);

        /* Invalidate D-Cache to ensure fresh values from RAM */
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc1_dma_buffer, sizeof(adc1_dma_buffer));
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc2_dma_buffer, sizeof(adc2_dma_buffer));
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc3_dma_buffer, sizeof(adc3_dma_buffer));

        sum_a += adc1_dma_buffer[0];
        sum_b += adc2_dma_buffer[0];
        sum_c += adc3_dma_buffer[0];
    }

    uint16_t avg_a = (uint16_t)(sum_a / CALIBRATION_SAMPLES);
    uint16_t avg_b = (uint16_t)(sum_b / CALIBRATION_SAMPLES);
    uint16_t avg_c = (uint16_t)(sum_c / CALIBRATION_SAMPLES);

    /* Sanity Check 1: Symmetric offset window check [1.15V to 2.15V] */
    if (avg_a < OFFSET_RAW_MIN || avg_a > OFFSET_RAW_MAX ||
        avg_b < OFFSET_RAW_MIN || avg_b > OFFSET_RAW_MAX ||
        avg_c < OFFSET_RAW_MIN || avg_c > OFFSET_RAW_MAX)
    {
        g_offsets_calibrated = false;
        return -EINVAL;
    }

    /* Sanity Check 2: Phase-to-phase imbalance check (max 100mV spread between sense lines) */
    uint16_t max_val = avg_a > avg_b ? (avg_a > avg_c ? avg_a : avg_c) : (avg_b > avg_c ? avg_b : avg_c);
    uint16_t min_val = avg_a < avg_b ? (avg_a < avg_c ? avg_a : avg_c) : (avg_b < avg_c ? avg_b : avg_c);
    if ((max_val - min_val) > 1986U) /* 1986 counts = ~100 mV */
    {
        g_offsets_calibrated = false;
        return -EILSEQ;
    }

    /* Store offsets in global structure */
    g_latest_currents.raw_offset_a = avg_a;
    g_latest_currents.raw_offset_b = avg_b;
    g_latest_currents.raw_offset_c = avg_c;
    g_offsets_calibrated = true;

    return 0;
}

static int cmd_read_dma_buffer(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct adc_info
    {
        const char *name;
        uint16_t *buffer;
        size_t size;
    } adcs[] = {
        {"ADC1", adc1_dma_buffer, sizeof(adc1_dma_buffer)},
        {"ADC2", adc2_dma_buffer, sizeof(adc2_dma_buffer)},
        {"ADC3", adc3_dma_buffer, sizeof(adc3_dma_buffer)},
    };

    for (size_t a = 0; a < ARRAY_SIZE(adcs); a++)
    {
        /* Invalidate D-Cache on STM32H7 for each DMA memory area before reading */
        SCB_InvalidateDCache_by_Addr((uint32_t *)adcs[a].buffer, adcs[a].size);

        shell_print(sh, "==========================================");
        shell_print(sh, "      %s DMA CIRCULAR BUFFER (8x)       ", adcs[a].name);
        shell_print(sh, "==========================================");

        for (int i = 0; i < ADC_BUFFER_SIZE; i++)
        {
            uint16_t raw = adcs[a].buffer[i];
            uint32_t mv = ((uint32_t)raw * 3300U) / 65535U;
            shell_print(sh, "Buffer[%d] : Raw %u (0x%04X) -> %u mV", i, raw, raw, mv);
        }

        shell_print(sh, "==========================================\n");
    }

    return 0;
}

SHELL_CMD_REGISTER(read_dma, NULL, "Read 1-element ADC1 DMA circular buffer", cmd_read_dma_buffer);

uint32_t manual_adc_read_single(ADC_HandleTypeDef *hadc)
{
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        return HAL_ADC_GetValue(hadc);
    }
    return 0;
}

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

static int cmd_calibrate_offsets(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Sampling %u samples for Phase A, B, C current offset...", CALIBRATION_SAMPLES);

    int ret = calibrate_current_offsets();
    if (ret == -EINVAL)
    {
        shell_error(sh, "CALIBRATION FAILED: Raw reading out of bounds [1150mV - 2150mV]!");
        return ret;
    }
    else if (ret == -EILSEQ)
    {
        shell_error(sh, "CALIBRATION FAILED: Excessive offset imbalance between phases (>100mV)!");
        return ret;
    }
    else if (ret < 0)
    {
        shell_error(sh, "CALIBRATION FAILED: ADC/DMA not running (Error %d)", ret);
        return ret;
    }

    uint32_t mv_a = raw_to_mv(g_latest_currents.raw_offset_a);
    uint32_t mv_b = raw_to_mv(g_latest_currents.raw_offset_b);
    uint32_t mv_c = raw_to_mv(g_latest_currents.raw_offset_c);

    shell_print(sh, "==========================================");
    shell_print(sh, "     CURRENT OFFSET CALIBRATION SUCCESS   ");
    shell_print(sh, "==========================================");
    shell_print(sh, "Phase A Offset : %u raw (%u mV)", g_latest_currents.raw_offset_a, mv_a);
    shell_print(sh, "Phase B Offset : %u raw (%u mV)", g_latest_currents.raw_offset_b, mv_b);
    shell_print(sh, "Phase C Offset : %u raw (%u mV)", g_latest_currents.raw_offset_c, mv_c);
    shell_print(sh, "==========================================\n");

    return 0;
}

SHELL_CMD_REGISTER(calibrate_offsets, NULL, "Calibrate zero-current ADC offsets", cmd_calibrate_offsets);

int cmd_capture_adc_csv(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const uint16_t total_samples = 1000;

    /* Print CSV Header */
    shell_print(sh, "SOA (raw), SOB (raw), SOC (raw)");

    for (uint16_t i = 0; i < total_samples; i++)
    {
        /* Wait for next DMA write / Timer trigger cycle */
        k_busy_wait(25);
        /* Invalidate D-Cache to ensure fresh values from RAM */
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc1_dma_buffer, sizeof(adc1_dma_buffer));
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc2_dma_buffer, sizeof(adc2_dma_buffer));
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc3_dma_buffer, sizeof(adc3_dma_buffer));

        /* Print CSV row */
        shell_print(sh, "%u, %u, %u", adc1_dma_buffer[0], adc2_dma_buffer[0], adc3_dma_buffer[0]);
    }

    /* Optional small delay (e.g. 1ms) if you want time-spaced sampling */
    /* k_msleep(1); */

return 0;
}

/* Register the command with the Zephyr Shell */
SHELL_CMD_ARG_REGISTER(capture_adc_csv, NULL,
                       "Capture 1000 ADC raw samples in CSV format",
                       cmd_capture_adc_csv, 1, 0);