/* ============================================================================
 * foc_calib.h — Encoder/electrical alignment, gain calibration, DQ offset.
 *
 * Interface:
 *   - calibrateMotor()              top-level; runs full per-motor sequence.
 *   - foc_calib_maybe_sample()      called from FOC ISR every tick during
 *                                   the alignment sweeps to log samples.
 * ============================================================================ */
#ifndef FOC_CALIB_H
#define FOC_CALIB_H

#include <stdint.h>
#include "foc_config.h"


/* Top-level: full calibration sequence for one motor. */
void calibrateMotor(uint8_t motor);


/* Called from the FOC ISR every cycle while in FOC_STATE_ALIGN_SWEEP.
 * Cheap when sampling is inactive (one-byte flag + branch). */
extern volatile uint8_t linreg_active;
void foc_calib_sample(uint8_t motor);

static inline void foc_calib_maybe_sample(uint8_t motor)
{
    if (linreg_active) foc_calib_sample(motor);
}


#endif /* FOC_CALIB_H */
