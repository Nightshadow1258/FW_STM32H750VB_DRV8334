/**
 * @file pwm_timer.h
 * @brief High Speed Center Aligned PWM Generation Interface for 3 Phase Inverters
 *
 * Configures STM32 TIM1 peripheral for center aligned complementary PWM drive 
 * with dead time insertion, output enable management, and TRGO generation for ADC trigger.
 */

#ifndef PWM_TIMER_H_
#define PWM_TIMER_H_

#include <zephyr/kernel.h>
#include <stm32_ll_tim.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration Constants */
#define PWM_TIMER_ARR_20KHZ_120MHZ   6000U  /**< ARR value for 20 kHz PWM @ 120 MHz clock */
#define PWM_TIMER_DEFAULT_DEAD_TIME  120U   /**< 500ns dead time at 120 MHz timer clock */
#define PWM_TIMER_MAX_DUTY_PCT       90U    /**< Maximum allowable duty cycle limit (%) */

/**
 * @brief Get pointer to the active TIM1 peripheral instance.
 *
 * Used by control modules (e.g., FOC controller) to set compare registers or 
 * read timer status registers without managing global state variables.
 *
 * @return TIM_TypeDef* Pointer to TIM1 register memory map
 */
TIM_TypeDef *pwm_timer_get_instance(void);

/**
 * @brief Initialize and start center-aligned 3-phase PWM generation on TIM1.
 *
 * Configures TIM1 CH1/CH1N, CH2/CH2N, CH3/CH3N for PWM Mode 1 with dead time,
 * automatic output enable (AOE), preloads enabled, and TRGO on Update Event.
 *
 * @return 0 on success, negative error code on failure.
 */
int pwm_timer_start_3phase_center_aligned(void);

/**
 * @brief Set compare raw ticks directly on all 3 PWM channels.
 *
 * @param ccr1 Ticks for Channel 1
 * @param ccr2 Ticks for Channel 2
 * @param ccr3 Ticks for Channel 3
 */
void pwm_timer_set_duty_ticks(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);

/**
 * @brief Safely disable all PWM outputs by clearing the Main Output Enable (MOE) bit.
 */
void pwm_timer_stop_outputs(void);

#ifdef __cplusplus
}
#endif

#endif /* PWM_TIMER_H_ */