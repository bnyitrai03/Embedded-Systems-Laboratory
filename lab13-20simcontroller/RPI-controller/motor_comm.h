#ifndef MOTOR_COMM_H
#define MOTOR_COMM_H

#include "control_protocol.h"

/*
 * Hardware boundary for the rest of the controller.
 *
 * Raspberry Pi SPI, a future DE10/Avalon implementation, or a desktop simulator
 * should all expose this same exchange operation: send the latest motor command
 * and receive the latest encoder sample.
 */
typedef struct MotorComm MotorComm;

typedef int (*MotorCommExchange)(void *context,
                                 MotorCommand command,
                                 EncoderSample *sample);
typedef void (*MotorCommClose)(void *context);

struct MotorComm {
    void *context;
    MotorCommExchange exchange;
    MotorCommClose close;
};

static inline int motor_comm_exchange(MotorComm *comm,
                                      MotorCommand command,
                                      EncoderSample *sample)
{
    return comm->exchange(comm->context, command, sample);
}

static inline void motor_comm_close(MotorComm *comm)
{
    if (comm->close != 0) {
        comm->close(comm->context);
    }
}

#endif
