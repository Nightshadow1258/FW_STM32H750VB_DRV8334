/**
 * @file pwm_timer.c
 * @brief STM32 TIM1 LL Peripheral Driver Implementation
 */

#include "pwm_timer.h"

#include <stdlib.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/shell/shell.h>

/* Devicetree Node Definitions */
#define TIM1_NODE DT_NODELABEL(timers1)

#if !DT_NODE_HAS_STATUS_OKAY(TIM1_NODE)
#error "Devicetree node 'timers1' is disabled or missing."
#endif

/* Encapsulated static reference to hardware peripheral */
static TIM_TypeDef *const tim1_instance = (TIM_TypeDef *)DT_REG_ADDR(TIM1_NODE);

TIM_TypeDef *pwm_timer_get_instance(void)
{
    return tim1_instance;
}

int pwm_timer_start_3phase_center_aligned(void)
{
    /* 1. Disable counter and clear main outputs during setup */
    LL_TIM_DisableCounter(tim1_instance);
    LL_TIM_DisableAllOutputs(tim1_instance);

    /* 2. Configure timebase: Center-aligned mode at 20 kHz */
    LL_TIM_SetPrescaler(tim1_instance, 0);
    LL_TIM_SetAutoReload(tim1_instance, PWM_TIMER_ARR_20KHZ_120MHZ);
    LL_TIM_SetCounterMode(tim1_instance, LL_TIM_COUNTERMODE_CENTER_UP_DOWN);

    /* 3. Initialize startup 50% duty cycle across all phases */
    const uint32_t mid_compare = PWM_TIMER_ARR_20KHZ_120MHZ / 2U;
    LL_TIM_OC_SetCompareCH1(tim1_instance, mid_compare);
    LL_TIM_OC_SetCompareCH2(tim1_instance, mid_compare);
    LL_TIM_OC_SetCompareCH3(tim1_instance, mid_compare);

    /* 4. Configure channels in PWM Mode 1 with shadow preloads enabled */
    LL_TIM_OC_SetMode(tim1_instance, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(tim1_instance, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(tim1_instance, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(tim1_instance, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(tim1_instance, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(tim1_instance, LL_TIM_CHANNEL_CH3);

    /* 5. Set active-high polarity for main and complementary channels */
    LL_TIM_OC_SetPolarity(tim1_instance, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1_instance, LL_TIM_CHANNEL_CH1N, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1_instance, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1_instance, LL_TIM_CHANNEL_CH2N, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1_instance, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1_instance, LL_TIM_CHANNEL_CH3N, LL_TIM_OCPOLARITY_HIGH);

    /* 6. Enable output channels */
    LL_TIM_CC_EnableChannel(tim1_instance, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(tim1_instance, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(tim1_instance, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(tim1_instance, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(tim1_instance, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(tim1_instance, LL_TIM_CHANNEL_CH3N);

    /* 7. Configure BDTR: Dead-time insertion and automatic output enable */
    LL_TIM_OC_SetDeadTime(tim1_instance, PWM_TIMER_DEFAULT_DEAD_TIME);
    LL_TIM_DisableBRK(tim1_instance);
    LL_TIM_EnableAutomaticOutput(tim1_instance);

    /* 8. Trigger setup: Emit TRGO pulse on Update Event for ADC sampling */
    LL_TIM_SetTriggerOutput(tim1_instance, LL_TIM_TRGO_UPDATE);
    LL_TIM_SetRepetitionCounter(tim1_instance, 0);

    /* 9. Commit shadow registers and start counter */
    LL_TIM_GenerateEvent_UPDATE(tim1_instance);
    LL_TIM_ClearFlag_UPDATE(tim1_instance);
    LL_TIM_EnableARRPreload(tim1_instance);

    LL_TIM_EnableAllOutputs(tim1_instance);
    LL_TIM_EnableCounter(tim1_instance);

    return 0;
}

void pwm_timer_set_duty_ticks(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
    LL_TIM_OC_SetCompareCH1(tim1_instance, ccr1);
    LL_TIM_OC_SetCompareCH2(tim1_instance, ccr2);
    LL_TIM_OC_SetCompareCH3(tim1_instance, ccr3);
}

void pwm_timer_stop_outputs(void)
{
    LL_TIM_DisableAllOutputs(tim1_instance);
    pwm_timer_set_duty_ticks(0, 0, 0);
}

/* =========================================================================
 * Shell Commands Section
 * ========================================================================= */

static int cmd_pwm_set(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 4) {
        shell_error(sh, "Usage: pwm set <duty_a_pct> <duty_b_pct> <duty_c_pct>");
        return -EINVAL;
    }

    uint32_t duty_a = strtoul(argv[1], NULL, 10);
    uint32_t duty_b = strtoul(argv[2], NULL, 10);
    uint32_t duty_c = strtoul(argv[3], NULL, 10);

    if (duty_a > PWM_TIMER_MAX_DUTY_PCT) duty_a = PWM_TIMER_MAX_DUTY_PCT;
    if (duty_b > PWM_TIMER_MAX_DUTY_PCT) duty_b = PWM_TIMER_MAX_DUTY_PCT;
    if (duty_c > PWM_TIMER_MAX_DUTY_PCT) duty_c = PWM_TIMER_MAX_DUTY_PCT;

    uint32_t arr = LL_TIM_GetAutoReload(tim1_instance);

    uint32_t compare_a = (duty_a * arr) / 100U;
    uint32_t compare_b = (duty_b * arr) / 100U;
    uint32_t compare_c = (duty_c * arr) / 100U;

    pwm_timer_set_duty_ticks(compare_a, compare_b, compare_c);

    if (!LL_TIM_IsEnabledAllOutputs(tim1_instance)) {
        LL_TIM_EnableAllOutputs(tim1_instance);
    }

    shell_print(sh, "PWM Duty set: A=%u%% (%u/%u), B=%u%% (%u/%u), C=%u%% (%u/%u)",
                duty_a, compare_a, arr,
                duty_b, compare_b, arr,
                duty_c, compare_c, arr);

    return 0;
}

static int cmd_pwm_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    pwm_timer_stop_outputs();
    shell_print(sh, "PWM Outputs Disabled (MOE cleared)");

    return 0;
}

static int cmd_pwm_dump(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "================ TIM1 Register Dump ================");
    shell_print(sh, "Base Address : 0x%08X", (uint32_t)tim1_instance);
    shell_print(sh, "----------------------------------------------------");

    shell_print(sh, "CR1          : 0x%08X (CEN=%d, CMS=%d)",
                tim1_instance->CR1,
                (tim1_instance->CR1 & TIM_CR1_CEN) ? 1 : 0,
                (tim1_instance->CR1 >> TIM_CR1_CMS_Pos) & 0x03);
    shell_print(sh, "CR2          : 0x%08X", tim1_instance->CR2);
    shell_print(sh, "SMCR         : 0x%08X", tim1_instance->SMCR);
    shell_print(sh, "DIER         : 0x%08X", tim1_instance->DIER);
    shell_print(sh, "SR           : 0x%08X (UIF=%d, BIF=%d)",
                tim1_instance->SR,
                (tim1_instance->SR & TIM_SR_UIF) ? 1 : 0,
                (tim1_instance->SR & TIM_SR_BIF) ? 1 : 0);

    shell_print(sh, "CCMR1        : 0x%08X (OC1M=%d, OC2M=%d)",
                tim1_instance->CCMR1,
                (tim1_instance->CCMR1 >> TIM_CCMR1_OC1M_Pos) & 0x07,
                (tim1_instance->CCMR1 >> TIM_CCMR1_OC2M_Pos) & 0x07);
    shell_print(sh, "CCMR2        : 0x%08X (OC3M=%d)",
                tim1_instance->CCMR2,
                (tim1_instance->CCMR2 >> TIM_CCMR2_OC3M_Pos) & 0x07);
    shell_print(sh, "CCER         : 0x%08X", tim1_instance->CCER);

    shell_print(sh, "CNT          : %u (0x%08X)", tim1_instance->CNT, tim1_instance->CNT);
    shell_print(sh, "PSC          : %u", tim1_instance->PSC);
    shell_print(sh, "ARR          : %u", tim1_instance->ARR);

    shell_print(sh, "CCR1         : %u", tim1_instance->CCR1);
    shell_print(sh, "CCR2         : %u", tim1_instance->CCR2);
    shell_print(sh, "CCR3         : %u", tim1_instance->CCR3);

    shell_print(sh, "BDTR         : 0x%08X (MOE=%u, AOE=%u, DTG=%u)",
                (unsigned int)((tim1_instance->BDTR & TIM_BDTR_MOE_Msk) >> TIM_BDTR_MOE_Pos),
                (unsigned int)((tim1_instance->BDTR & TIM_BDTR_AOE_Msk) >> TIM_BDTR_AOE_Pos),
                (unsigned int)(tim1_instance->BDTR & TIM_BDTR_DTG_Msk));

    shell_print(sh, "====================================================");

    return 0;
}

static int cmd_pwm_freq(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));
    uint32_t rate = 0;

    struct stm32_pclken pclken = {
        .bus = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(timers1), 0, bus),
        .enr = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(timers1), 0, bits)
    };

    if (clock_control_get_rate(clk_dev, (clock_control_subsys_t)&pclken, &rate) == 0) {
        shell_print(sh, "TIM1 Input Clock Rate: %u Hz", rate);
    } else {
        shell_error(sh, "Failed to fetch TIM1 clock rate from RCC");
    }

    return 0;
}

/* Register Subcommands under "pwm" root command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_pwm,
    SHELL_CMD_ARG(set, NULL, "Set PWM duty (0..90%%)\nUsage: pwm set <A> <B> <C>", cmd_pwm_set, 4, 0),
    SHELL_CMD(stop, NULL, "Disable PWM outputs (Clear MOE)", cmd_pwm_stop),
    SHELL_CMD(dump, NULL, "Dump TIM1 core registers", cmd_pwm_dump),
    SHELL_CMD(freq, NULL, "Print TIM1 clock frequency", cmd_pwm_freq),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pwm, &sub_pwm, "PWM Driver Management Commands", NULL);