#include "pwm_timer.h"

static TIM_TypeDef *const tim1 = (TIM_TypeDef *)DT_REG_ADDR(TIM1_NODE);

int start_center_aligned_3phase_pwm(void)
{
    // 2. Stop counter and disable outputs for setup
    LL_TIM_DisableCounter(tim1);
    LL_TIM_DisableAllOutputs(tim1);

    // 3. Set prescaler, auto-reload (20 kHz center aligned @ 120 MHz), and mode
    LL_TIM_SetPrescaler(tim1, 0);
    LL_TIM_SetAutoReload(tim1, 6000); // 20 kHz center-aligned PWM with 120 MHz timer clock 
    LL_TIM_SetCounterMode(tim1, LL_TIM_COUNTERMODE_CENTER_UP_DOWN);

    // 4. Set 50% duty cycle on all 3 channels
    LL_TIM_OC_SetCompareCH1(tim1, 3000);
    LL_TIM_OC_SetCompareCH2(tim1, 2000);
    LL_TIM_OC_SetCompareCH3(tim1, 4000);

    // 5. Configure PWM Mode 1 with shadow preloads
    LL_TIM_OC_SetMode(tim1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(tim1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(tim1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(tim1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(tim1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(tim1, LL_TIM_CHANNEL_CH3);

    // 6. Set active high output polarities
    LL_TIM_OC_SetPolarity(tim1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1, LL_TIM_CHANNEL_CH1N, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1, LL_TIM_CHANNEL_CH2N, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(tim1, LL_TIM_CHANNEL_CH3N, LL_TIM_OCPOLARITY_HIGH);

    // 7. Enable physical channels
    LL_TIM_CC_EnableChannel(tim1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(tim1, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(tim1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(tim1, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(tim1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(tim1, LL_TIM_CHANNEL_CH3N);

    // 8. Configure BDTR: Set Dead-time, disable breaks, enable AOE
    LL_TIM_OC_SetDeadTime(tim1, 120); // 500ns dead-time at 120 MHz 
    LL_TIM_DisableBRK(tim1);
    LL_TIM_EnableAutomaticOutput(tim1);

    // Configure TIM1 to send TRGO pulse on Update Event
    LL_TIM_SetTriggerOutput(tim1, LL_TIM_TRGO_UPDATE); //should be used to trigger ADC conversions

    // Set Repetition Counter to 1 so Update Event fires once per full up down cycle
    LL_TIM_SetRepetitionCounter(tim1, 0);

    // Generate software update event to reload RCR into shadow register
    LL_TIM_GenerateEvent_UPDATE(tim1);

    // 9. Load preload values into active shadow registers
    LL_TIM_ClearFlag_UPDATE(tim1);

    LL_TIM_EnableARRPreload(tim1);
    // 10. Enable main output (MOE bit) and start counter
    LL_TIM_EnableAllOutputs(tim1);
    LL_TIM_EnableCounter(tim1);
    return 0;
}

static int cmd_dump_tim1(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    TIM_TypeDef *tim1 = (TIM_TypeDef *)DT_REG_ADDR(TIM1_NODE);

    shell_print(sh, "================ TIM1 Register Dump ================");
    shell_print(sh, "Base Address : 0x%08X", (uint32_t)tim1);
    shell_print(sh, "----------------------------------------------------");

    /* Control and Status Registers */
    shell_print(sh, "CR1          : 0x%08X (CEN=%d, CMS=%d)",
                tim1->CR1,
                (tim1->CR1 & TIM_CR1_CEN) ? 1 : 0,
                (tim1->CR1 >> TIM_CR1_CMS_Pos) & 0x03);
    shell_print(sh, "CR2          : 0x%08X", tim1->CR2);
    shell_print(sh, "SMCR         : 0x%08X", tim1->SMCR);
    shell_print(sh, "DIER         : 0x%08X", tim1->DIER);
    shell_print(sh, "SR           : 0x%08X (UIF=%d, BIF=%d)",
                tim1->SR,
                (tim1->SR & TIM_SR_UIF) ? 1 : 0,
                (tim1->SR & TIM_SR_BIF) ? 1 : 0);
    shell_print(sh, "EGR          : 0x%08X", tim1->EGR);

    /* Capture/Compare Selection and Enable Registers */
    shell_print(sh, "CCMR1        : 0x%08X (OC1M=%d, OC2M=%d)",
                tim1->CCMR1,
                (tim1->CCMR1 >> TIM_CCMR1_OC1M_Pos) & 0x07,
                (tim1->CCMR1 >> TIM_CCMR1_OC2M_Pos) & 0x07);
    shell_print(sh, "CCMR2        : 0x%08X (OC3M=%d)",
                tim1->CCMR2,
                (tim1->CCMR2 >> TIM_CCMR2_OC3M_Pos) & 0x07);
    shell_print(sh, "CCER         : 0x%08X (CC1E=%d, CC1NE=%d, CC2E=%d, CC2NE=%d, CC3E=%d, CC3NE=%d)",
                tim1->CCER,
                (tim1->CCER & TIM_CCER_CC1E) ? 1 : 0,
                (tim1->CCER & TIM_CCER_CC1NE) ? 1 : 0,
                (tim1->CCER & TIM_CCER_CC2E) ? 1 : 0,
                (tim1->CCER & TIM_CCER_CC2NE) ? 1 : 0,
                (tim1->CCER & TIM_CCER_CC3E) ? 1 : 0,
                (tim1->CCER & TIM_CCER_CC3NE) ? 1 : 0);

    /* Counter and Reload Registers */
    shell_print(sh, "CNT          : %u (0x%08X)", tim1->CNT, tim1->CNT);
    shell_print(sh, "PSC          : %u", tim1->PSC);
    shell_print(sh, "ARR          : %u", tim1->ARR);

    /* Channel Compare Registers */
    shell_print(sh, "CCR1         : %u", tim1->CCR1);
    shell_print(sh, "CCR2         : %u", tim1->CCR2);
    shell_print(sh, "CCR3         : %u", tim1->CCR3);

    /* Break and Dead-Time / Output Enable Register */
    shell_print(sh, "BDTR          : 0x%08X (MOE=%u, AOE=%u, DTG=%u)",
            (unsigned int)((tim1->BDTR & TIM_BDTR_MOE_Msk) >> TIM_BDTR_MOE_Pos),
            (unsigned int)((tim1->BDTR & TIM_BDTR_AOE_Msk) >> TIM_BDTR_AOE_Pos),
            (unsigned int)(tim1->BDTR & TIM_BDTR_DTG_Msk));

    shell_print(sh, "====================================================");

    return 0;
}

/* Register the root shell command "tim1_dump" */
SHELL_CMD_REGISTER(tim1_dump, NULL, "Dump TIM1 core and channel registers", cmd_dump_tim1);

void print_tim1_clock_frequency(const struct shell *sh)
{
    const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));
    uint32_t rate = 0;

    struct stm32_pclken pclken = {
        .bus = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(timers1), 0, bus),
        .enr = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(timers1), 0, bits)};

    if (clock_control_get_rate(clk_dev, (clock_control_subsys_t)&pclken, &rate) == 0)
    {
        shell_print(sh, "TIM1 Input Clock Rate: %u Hz", rate);
    }
    else
    {
        shell_print(sh, "Failed to fetch TIM1 clock rate");
    }
}

SHELL_CMD_REGISTER(tim1_clock_freq, NULL, "Print TIM1 clock frequency", print_tim1_clock_frequency);


static int cmd_set_pwm_duty(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 4) {
        shell_error(sh, "Usage: set_pwm <duty_a_pct> <duty_b_pct> <duty_c_pct>");
        return -EINVAL;
    }

    uint32_t duty_a = strtoul(argv[1], NULL, 10);
    uint32_t duty_b = strtoul(argv[2], NULL, 10);
    uint32_t duty_c = strtoul(argv[3], NULL, 10);

    /* Clamp duty cycle bounds between 0% and 90% */
    if (duty_a > 90) duty_a = 90;
    if (duty_b > 90) duty_b = 90;
    if (duty_c > 90) duty_c = 90;

    uint32_t arr = LL_TIM_GetAutoReload(tim1);

    uint32_t compare_a = (duty_a * arr) / 100U;
    uint32_t compare_b = (duty_b * arr) / 100U;
    uint32_t compare_c = (duty_c * arr) / 100U;

    LL_TIM_OC_SetCompareCH1(tim1, compare_a);
    LL_TIM_OC_SetCompareCH2(tim1, compare_b);
    LL_TIM_OC_SetCompareCH3(tim1, compare_c);

    if (!LL_TIM_IsEnabledAllOutputs(tim1)) {
        LL_TIM_EnableAllOutputs(tim1);
    }

    shell_print(sh, "PWM Duty set: A=%u%% (%u/%u), B=%u%% (%u/%u), C=%u%% (%u/%u)",
                duty_a, compare_a, arr,
                duty_b, compare_b, arr,
                duty_c, compare_c, arr);

    return 0;
}

/* Command to disable all PWM outputs */
static int cmd_pwm_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    LL_TIM_DisableAllOutputs(tim1);
    
    LL_TIM_OC_SetCompareCH1(tim1, 0);
    LL_TIM_OC_SetCompareCH2(tim1, 0);
    LL_TIM_OC_SetCompareCH3(tim1, 0);

    shell_print(sh, "PWM Outputs Disabled (MOE cleared)");
    return 0;
}

/* Register commands with Zephyr Shell */
SHELL_CMD_ARG_REGISTER(set_pwm, NULL, 
                       "Set Phase A, B, C duty cycles in %% (0 to 90)\nUsage: set_pwm <A> <B> <C>", 
                       cmd_set_pwm_duty, 4, 0);

SHELL_CMD_REGISTER(stop_pwm, NULL, 
                   "Disable PWM outputs (Clear MOE)", 
                   cmd_pwm_stop);