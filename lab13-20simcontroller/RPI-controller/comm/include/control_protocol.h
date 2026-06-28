#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include <stdint.h>
#include "jiwy_config.h"

/**
 * Frame used between the control application and the FPGA.
 *
 * TX, big-endian:
 *   yaw   bit15 direction, bit14 enable, bit13..0 PWM
 *   pitch bit15 direction, bit14 enable, bit13..0 PWM
 *
 * RX, big-endian:
 *   yaw signed int16 encoder count, then pitch signed int16 encoder count.
 */

typedef enum {
    /** Move toward decreasing encoder counts. */
    MOTOR_DIR_NEGATIVE = 0,
    MOTOR_DIR_POSITIVE = 1
} MotorDirection;

typedef struct {
    /** Direction bit */
    MotorDirection direction;
    /** Nonzero enables PWM output */
    uint8_t enable;
    /** PWM magnitude */
    uint16_t pwm;
} AxisCommand;

typedef struct {
    AxisCommand yaw;
    AxisCommand pitch;
} MotorCommand;

typedef struct {
    int16_t yaw;
    int16_t pitch;
} EncoderSample;

/**
 * @return Packed command word with direction, enable, and PWM bits.
 */
uint16_t protocol_pack_axis(AxisCommand command);

/**
 * @return Axis command with enable cleared and zero PWM.
 */
AxisCommand protocol_axis_off(void);

/**
 * @return Motor command that disables yaw and pitch.
 */
MotorCommand protocol_stop_command(void);

/**
 * @param command Motor command.
 * @param tx Destination byte array, yaw first and pitch second.
 */
void protocol_pack_command(MotorCommand command, uint8_t tx[4]);

/**
 * @brief Unpack the 4-byte SPI RX frame into signed encoder counts.
 */
EncoderSample protocol_unpack_encoders(const uint8_t rx[4]);

#endif
