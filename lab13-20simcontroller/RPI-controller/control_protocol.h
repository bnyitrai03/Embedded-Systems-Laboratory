#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include <stdint.h>
#include "jiwy_config.h"
/**
 * @file control_protocol.h
 * @brief FPGA motor command and encoder sample wire format helpers.
 *
 * Shared motor/encoder frame used between the control application and the FPGA.
 *
 * TX, big-endian:
 *   yaw   bit15 direction, bit14 enable, bit13..0 PWM
 *   pitch bit15 direction, bit14 enable, bit13..0 PWM
 *
 * RX, big-endian:
 *   yaw signed int16 encoder count, then pitch signed int16 encoder count.
 */



/** @brief Motor direction bit convention used by the FPGA protocol. */
typedef enum {
    /** Move toward decreasing encoder counts. */
    MOTOR_DIR_NEGATIVE = 0,
    /** Move toward increasing encoder counts. */
    MOTOR_DIR_POSITIVE = 1
} MotorDirection;

/** @brief Command for one motor axis before protocol packing. */
typedef struct {
    /** Direction bit to send when the axis is enabled. */
    MotorDirection direction;
    /** Nonzero enables PWM output for this axis. */
    uint8_t enable;
    /** PWM magnitude, clamped to MOTOR_PWM_MAX when packed. */
    uint16_t pwm;
} AxisCommand;

/** @brief Command for both JIWY axes. */
typedef struct {
    /** Yaw/pan motor command. */
    AxisCommand yaw;
    /** Pitch/tilt motor command. */
    AxisCommand pitch;
} MotorCommand;

/** @brief Signed encoder counts received from the FPGA. */
typedef struct {
    /** Yaw/pan encoder count. */
    int16_t yaw;
    /** Pitch/tilt encoder count. */
    int16_t pitch;
} EncoderSample;

/**
 * @brief Pack one axis command into the 16-bit FPGA command word.
 * @param command Axis command in local struct form.
 * @return Packed command word with direction, enable, and PWM bits.
 */
uint16_t protocol_pack_axis(AxisCommand command);

/**
 * @brief Return a disabled axis command.
 * @return Axis command with enable cleared and zero PWM.
 */
AxisCommand protocol_axis_off(void);

/**
 * @brief Return a disabled two-axis motor command.
 * @return Motor command that disables yaw and pitch.
 */
MotorCommand protocol_stop_command(void);

/**
 * @brief Pack a two-axis motor command into the 4-byte SPI TX frame.
 * @param command Local motor command.
 * @param tx Destination byte array, yaw first and pitch second.
 */
void protocol_pack_command(MotorCommand command, uint8_t tx[4]);

/**
 * @brief Unpack the 4-byte SPI RX frame into signed encoder counts.
 * @param rx Source byte array, yaw first and pitch second.
 * @return Decoded encoder sample.
 */
EncoderSample protocol_unpack_encoders(const uint8_t rx[4]);

#endif
