#include "control_protocol.h"

uint16_t protocol_pack_axis(AxisCommand command)
{
    uint16_t pwm = command.pwm;

    /* Clamp here before bits go to the FPGA. */
    if (pwm > MOTOR_PWM_MAX) {
        pwm = MOTOR_PWM_MAX;
    }

    return (uint16_t)(((command.direction == MOTOR_DIR_POSITIVE) ? 0x8000u : 0u) | (command.enable ? 0x4000u : 0u) | pwm);
}

AxisCommand protocol_axis_off(void)
{
    AxisCommand command;

    command.direction = MOTOR_DIR_NEGATIVE;
    command.enable = 0;
    command.pwm = 0;
    return command;
}

MotorCommand protocol_stop_command(void)
{
    MotorCommand command;

    command.yaw = protocol_axis_off();
    command.pitch = protocol_axis_off();
    return command;
}

void protocol_pack_command(MotorCommand command, uint8_t tx[4])
{
    uint16_t yaw = protocol_pack_axis(command.yaw);
    uint16_t pitch = protocol_pack_axis(command.pitch);

    tx[0] = (uint8_t)(yaw >> 8);
    tx[1] = (uint8_t)(yaw & 0xFFu);
    tx[2] = (uint8_t)(pitch >> 8);
    tx[3] = (uint8_t)(pitch & 0xFFu);
}

EncoderSample protocol_unpack_encoders(const uint8_t rx[4])
{
    EncoderSample sample;
    uint16_t yaw = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    uint16_t pitch = (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);

    sample.yaw = (int16_t)yaw;
    sample.pitch = (int16_t)pitch;
    return sample;
}
