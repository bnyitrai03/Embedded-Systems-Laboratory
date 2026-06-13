#ifndef MOTOR_COMM_H
#define MOTOR_COMM_H

#include "comm/include/control_protocol.h"

/**
 * @file motor_comm.h
 * @brief Hardware communication abstraction used by homing and control loops.
 *
 * Hardware boundary for the rest of the controller.
 *
 * Raspberry Pi SPI, a future DE10/Avalon implementation, or a desktop simulator
 * should all expose this same exchange operation: send the latest motor command
 * and receive the latest encoder sample.
 */
typedef struct MotorComm MotorComm;

/**
 * @brief Exchange one motor command for one encoder sample.
 * @param context Backend-specific communication object.
 * @param command Command to send during this transaction.
 * @param sample Optional destination for received encoders; may be NULL.
 * @return 0 on success or a negative errno-style value on failure.
 */
typedef int (*MotorCommExchange)(void *context,
                                 MotorCommand command,
                                 EncoderSample *sample);

/**
 * @brief Close a backend communication context.
 * @param context Backend-specific communication object.
 */
typedef void (*MotorCommClose)(void *context);

/** @brief Runtime communication backend vtable and context. */
struct MotorComm {
    /** Backend-specific object passed to exchange and close. */
    void *context;
    /** Full-duplex command/sample exchange operation. */
    MotorCommExchange exchange;
    /** Optional close operation. */
    MotorCommClose close;
};

/**
 * @brief Dispatch one command/sample exchange through the active backend.
 */
static inline int motor_comm_exchange(MotorComm *comm,
                                      MotorCommand command,
                                      EncoderSample *sample)
{
    return comm->exchange(comm->context, command, sample);
}

/**
 * @brief Close the active backend if it provided a close operation.
 */
static inline void motor_comm_close(MotorComm *comm)
{
    if (comm->close != 0) {
        comm->close(comm->context);
    }
}

#endif
