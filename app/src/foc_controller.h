#ifndef FOC_CONTROLLER_H_
#define FOC_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "pwm_timer.h"

struct open_loop_config {
    bool enabled;
    float frequency_hz;   /* Electrical frequency in Hz */
    float amplitude_pct;  /* Voltage vector amplitude (0.0 to 30.0%) */
    float angle_rad;      /* Current electrical angle in radians */
};

/**
 * @brief Retrieve current open-loop state configuration.
 */
const struct open_loop_config *foc_get_open_loop_config(void);

/**
 * @brief Enable or update open-loop motor drive.
 *
 * @param frequency_hz Electrical frequency in Hz.
 * @param amplitude_pct Voltage amplitude percentage (clamped to safety limit).
 * @return 0 on success, negative error code on invalid parameters.
 */
int foc_open_loop_start(float frequency_hz, float amplitude_pct);

/**
 * @brief Disable open-loop drive and safely stop PWM output.
 */
void foc_open_loop_stop(void);

/**
 * @brief Step open-loop angle and update PWM register compare values.
 *
 * Designed to be called continuously from a periodic thread (e.g. 1 kHz) 
 * or directly inside the TIM1 TRGO / ADC interrupt (e.g. 20 kHz).
 *
 * @param delta_time_s Execution period step in seconds.
 */
void foc_open_loop_update(float delta_time_s);

#endif /* FOC_CONTROLLER_H_ */