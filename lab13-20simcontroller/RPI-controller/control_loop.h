#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <signal.h>

#include "jiwy_calibration.h"
#include "motor_comm.h"
#include "twentysim_controller.h"

typedef struct {
    double yaw_target_rad;
    double pitch_target_rad;
} ControlTarget;

typedef struct {
    unsigned sample_period_us;
    unsigned log_period_samples;
} ControlLoopConfig;

typedef int (*ControlTargetProvider)(void *context,
                                     const JiwyCalibration *calibration,
                                     EncoderSample encoders,
                                     ControlTarget *target);

/*
 * Target providers are the extension point for future behavior:
 * fixed-position hold, joystick/manual input, trajectory playback, or vision.
 * They should be non-blocking so the 20-sim controller keeps its sample period.
 */
ControlLoopConfig control_loop_default_config(void);
int control_loop_fixed_target(void *context,
                              const JiwyCalibration *calibration,
                              EncoderSample encoders,
                              ControlTarget *target);
int control_loop_run(MotorComm *comm,
                     TwentySimController *controller,
                     const JiwyCalibration *calibration,
                     const ControlLoopConfig *config,
                     ControlTargetProvider target_provider,
                     void *target_context,
                     volatile sig_atomic_t *keep_running);

#endif
