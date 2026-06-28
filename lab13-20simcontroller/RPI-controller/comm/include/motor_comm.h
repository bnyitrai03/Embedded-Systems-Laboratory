#ifndef MOTOR_COMM_H
#define MOTOR_COMM_H

#include "comm/include/control_protocol.h"

/**
 * @brief Hardware communication abstraction used by homing and control loops.
 */
typedef struct MotorComm MotorComm;

/**
 * @brief Exchange one motor command for one encoder sample.
 * @param context Backend-specific communication object.
 * @param command Command to send during this transaction.
 * @param sample Destination for received encoders; may be NULL.
 * @return 0 on success or a negative errno-style value on failure.
 */
typedef int (*MotorCommExchange)(void *context, MotorCommand command, EncoderSample *sample);

/**
 * @brief Close communication.
 */
typedef void (*MotorCommClose)(void *context);

/** @brief Runtime communication backend and context. */
struct MotorComm {
    /** Backend-specific object passed to exchange and close. */
    void *context;
    /** Full-duplex command exchange operation. */
    MotorCommExchange exchange;
    /** Optional close operation. */
    MotorCommClose close;
};

/**
 * @brief Dispatch a command through the active backend.
 */
static inline int motor_comm_exchange(MotorComm *comm, MotorCommand command, EncoderSample *sample)
{
    return comm->exchange(comm->context, command, sample);
}

/**
 * @brief Close the active backend.
 */
static inline void motor_comm_close(MotorComm *comm)
{
    if (comm->close != 0) {
        comm->close(comm->context);
    }
}

#endif
