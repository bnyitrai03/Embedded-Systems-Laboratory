#ifndef RPI_SPI_MASTER_H
#define RPI_SPI_MASTER_H

#include "jiwy_config.h"
#include "comm/include/motor_comm.h"

/**
 * @brief Raspberry Pi spidev backend for the MotorComm interface.
 */

#define RPI_SPI_DEFAULT_SPEED_HZ JIWY_SPI_DEFAULT_SPEED_HZ
#define RPI_SPI_DEFAULT_CHANNEL JIWY_SPI_DEFAULT_CHANNEL

typedef struct {
    /** Open spidev file descriptor, or -1 when closed. */
    int fd;
    unsigned speed_hz;
    unsigned channel;
} RpiSpiComm;

/**
 * @brief Open and bind Raspberry Pi spidev backend to MotorComm.
 */
int rpi_spi_comm_open(RpiSpiComm *spi, MotorComm *comm, unsigned channel, unsigned speed_hz, unsigned spi_flags);

/**
 * @brief Close an open Raspberry Pi spidev backend.
 */
void rpi_spi_comm_close(RpiSpiComm *spi);

#endif
