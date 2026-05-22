#ifndef RPI_SPI_MASTER_H
#define RPI_SPI_MASTER_H

#include "motor_comm.h"

#define RPI_SPI_DEFAULT_SPEED_HZ 100000u
#define RPI_SPI_DEFAULT_CHANNEL 1u

/* spidev-backed implementation of MotorComm for Raspberry Pi + FPGA HAT. */
typedef struct {
    int fd;
    unsigned speed_hz;
    unsigned channel;
} RpiSpiComm;

int rpi_spi_comm_open(RpiSpiComm *spi,
                      MotorComm *comm,
                      unsigned channel,
                      unsigned speed_hz,
                      unsigned spi_flags);
void rpi_spi_comm_close(RpiSpiComm *spi);

#endif
