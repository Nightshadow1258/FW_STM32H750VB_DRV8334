#include "foc_controller.h"
#include "pwm_timer.h"  /* For LL_TIM_GetAutoReload and tim1 reference */
#include <math.h>
#include <stm32h7xx_ll_tim.h>
#include <stdlib.h>
#include <zephyr/shell/shell.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TWO_PI (2.0f * M_PI)
#define OPEN_LOOP_MAX_AMP_PCT 70.0f  /* Hard safety clamp for open-loop drive */

static struct open_loop_config g_open_loop = {
    .enabled = false,
    .frequency_hz = 1.0f,
    .amplitude_pct = 5.0f,
    .angle_rad = 0.0f
};

const struct open_loop_config *foc_get_open_loop_config(void)
{
    return &g_open_loop;
}

int foc_open_loop_start(float frequency_hz, float amplitude_pct)
{
    if (amplitude_pct <= 0.0f || frequency_hz == 0.0f) {
        foc_open_loop_stop();
        return 0;
    }

    if (amplitude_pct > OPEN_LOOP_MAX_AMP_PCT) {
        amplitude_pct = OPEN_LOOP_MAX_AMP_PCT;
    }

    g_open_loop.frequency_hz = frequency_hz;
    g_open_loop.amplitude_pct = amplitude_pct;
    g_open_loop.enabled = true;

    TIM_TypeDef *tim1 = pwm_timer_get_instance();
    if (!LL_TIM_IsEnabledAllOutputs(tim1)) {
        LL_TIM_EnableAllOutputs(tim1);
    }

    return 0;
}

void foc_open_loop_stop(void)
{
    g_open_loop.enabled = false;
    pwm_timer_stop_outputs();
}

void foc_open_loop_update(float delta_time_s)
{
    if (!g_open_loop.enabled) {
        return;
    }

    /* Advance electrical angle */
    g_open_loop.angle_rad += TWO_PI * g_open_loop.frequency_hz * delta_time_s;
    if (g_open_loop.angle_rad >= TWO_PI) {
        g_open_loop.angle_rad -= TWO_PI;
    } else if (g_open_loop.angle_rad < 0.0f) {
        g_open_loop.angle_rad += TWO_PI;
    }

    /* Sinusoidal phase voltage generation */
    float v_a = sinf(g_open_loop.angle_rad);
    float v_b = sinf(g_open_loop.angle_rad - (TWO_PI / 3.0f));
    float v_c = sinf(g_open_loop.angle_rad + (TWO_PI / 3.0f));

    /* Scale amplitude with 50% DC offset */
    float offset = 50.0f;
    float scale = g_open_loop.amplitude_pct / 2.0f;

    float duty_a = offset + (v_a * scale);
    float duty_b = offset + (v_b * scale);
    float duty_c = offset + (v_c * scale);

    TIM_TypeDef *tim1 = pwm_timer_get_instance();
    uint32_t arr = LL_TIM_GetAutoReload(tim1);

    uint32_t compare_a = (uint32_t)((duty_a / 100.0f) * (float)arr);
    uint32_t compare_b = (uint32_t)((duty_b / 100.0f) * (float)arr);
    uint32_t compare_c = (uint32_t)((duty_c / 100.0f) * (float)arr);

    pwm_timer_set_duty_ticks(compare_a, compare_b, compare_c);
}

/* =========================================================================
 * FOC Shell Command Group
 * ========================================================================= */

static int cmd_spin_start(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_error(sh, "Usage: spin start <freq_hz> <amp_pct>");
        shell_print(sh, "Example: spin start 2.0 10.0   (2 Hz electrical, 10%% amplitude)");
        return -EINVAL;
    }

    float freq = strtof(argv[1], NULL);
    float amp = strtof(argv[2], NULL);

    if (amp > OPEN_LOOP_MAX_AMP_PCT) {
        shell_warn(sh, "Amplitude limited to %.1f%% for current protection", (double)OPEN_LOOP_MAX_AMP_PCT);
    }

    int err = foc_open_loop_start(freq, amp);
    if (err == 0 && amp > 0.0f && freq != 0.0f) {
        shell_print(sh, "Spinning open loop: Freq = %.2f Hz, Amplitude = %.1f%%",
                    (double)freq, (double)amp);
    } else {
        shell_print(sh, "Open-loop drive stopped");
    }

    return 0;
}

static int cmd_spin_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    foc_open_loop_stop();
    shell_print(sh, "Open-loop drive disabled and PWM outputs cleared");

    return 0;
}

static int cmd_spin_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const struct open_loop_config *cfg = foc_get_open_loop_config();

    shell_print(sh, "================ Open-Loop State ================");
    shell_print(sh, "Status    : %s", cfg->enabled ? "ACTIVE" : "DISABLED");
    shell_print(sh, "Frequency : %.2f Hz", (double)cfg->frequency_hz);
    shell_print(sh, "Amplitude : %.1f%%", (double)cfg->amplitude_pct);
    shell_print(sh, "Angle     : %.2f rad", (double)cfg->angle_rad);
    shell_print(sh, "=================================================");

    return 0;
}

/* Subcommand hierarchy under "spin" namespace */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_spin,
    SHELL_CMD_ARG(start, NULL, "Start open loop spin\nUsage: spin start <freq_hz> <amp_pct>", cmd_spin_start, 3, 0),
    SHELL_CMD(stop, NULL, "Stop open loop spin", cmd_spin_stop),
    SHELL_CMD(status, NULL, "Display active open loop parameters", cmd_spin_status),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(spin, &sub_spin, "Open Loop Motor Control Commands", NULL);