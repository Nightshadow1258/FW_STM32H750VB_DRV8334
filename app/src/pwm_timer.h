#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <stm32_ll_tim.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#define TIM1_NODE DT_NODELABEL(timers1)

int start_center_aligned_3phase_pwm(void);
int init_center_aligned_timer(void);
