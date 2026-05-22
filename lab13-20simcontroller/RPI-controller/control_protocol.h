#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include <stdint.h>

/*
 * Shared motor/encoder frame used between the control application and the FPGA.
 *
 * TX, big-endian:
 *   yaw   bit15 direction, bit14 enable, bit13..0 PWM
 *   pitch bit15 direction, bit14 enable, bit13..0 PWM
 *
 * RX, big-endian:
 *   yaw signed int16 encoder count, then pitch signed int16 encoder count.
 */
/*
 * The protocol field is 14 bits, but the current FPGA PWM period is 2500 ticks
 * at 50 MHz / 20 kHz. Clamp commands to that hardware duty range on the C side.
 */
#define MOTOR_PWM_MAX 2500u

typedef enum {
    MOTOR_DIR_NEGATIVE = 0,
    MOTOR_DIR_POSITIVE = 1
} MotorDirection;

typedef struct {
    MotorDirection direction;
    uint8_t enable;
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

uint16_t protocol_pack_axis(AxisCommand command);
AxisCommand protocol_axis_off(void);
MotorCommand protocol_stop_command(void);
void protocol_pack_command(MotorCommand command, uint8_t tx[4]);
EncoderSample protocol_unpack_encoders(const uint8_t rx[4]);

#endif
