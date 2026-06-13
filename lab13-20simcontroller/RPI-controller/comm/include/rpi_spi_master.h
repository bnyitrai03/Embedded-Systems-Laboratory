#ifndef RPI_SPI_MASTER_H
#define RPI_SPI_MASTER_H

#include "jiwy_config.h"
#include "comm/include/motor_comm.h"

/**
 * @file rpi_spi_master.h
 * @brief Raspberry Pi spidev backend for the MotorComm interface.
 */

/** @brief Default SPI clock used for the FPGA motor controller. */
#define RPI_SPI_DEFAULT_SPEED_HZ JIWY_SPI_DEFAULT_SPEED_HZ

/** @brief Default Raspberry Pi SPI channel for the FPGA HAT. */
#define RPI_SPI_DEFAULT_CHANNEL JIWY_SPI_DEFAULT_CHANNEL

/** @brief spidev-backed implementation state for Raspberry Pi + FPGA HAT. */
typedef struct {
    /** Open spidev file descriptor, or -1 when closed. */
    int fd;
    /** Configured SPI bus speed in Hz. */
    unsigned speed_hz;
    /** Raspberry Pi SPI channel number. */
    unsigned channel;
} RpiSpiComm;

/**
 * @brief Open and bind a Raspberry Pi spidev backend to MotorComm.
 * @param spi Backend state to initialize.
 * @param comm Generic communication interface to bind.
 * @param channel Raspberry Pi SPI channel.
 * @param speed_hz SPI clock speed in Hz.
 * @param spi_flags Low bits select SPI mode flags.
 * @return 0 on success or a negative errno-style value on failure.
 */
int rpi_spi_comm_open(RpiSpiComm *spi,
                      MotorComm *comm,
                      unsigned channel,
                      unsigned speed_hz,
                      unsigned spi_flags);

/**
 * @brief Close an open Raspberry Pi spidev backend.
 * @param spi Backend state to close.
 */
void rpi_spi_comm_close(RpiSpiComm *spi);

#endif
