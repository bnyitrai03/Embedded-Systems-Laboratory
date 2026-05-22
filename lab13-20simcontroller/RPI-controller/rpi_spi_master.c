#include "rpi_spi_master.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int spi_open_device(unsigned spi_channel,
                           unsigned speed_hz,
                           unsigned spi_flags)
{
    int fd;
    uint8_t spi_mode = (uint8_t)(spi_flags & 0x3u);
    uint8_t spi_bits = 8;
    char dev[32];

    snprintf(dev, sizeof(dev), "/dev/spidev0.%u", spi_channel);

    fd = open(dev, O_RDWR);
    if (fd < 0) {
        return -errno;
    }

    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        int saved_errno = errno;
        close(fd);
        return -saved_errno;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) < 0) {
        int saved_errno = errno;
        close(fd);
        return -saved_errno;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
        int saved_errno = errno;
        close(fd);
        return -saved_errno;
    }

    return fd;
}

static int spi_transfer(RpiSpiComm *spi, uint8_t tx[4], uint8_t rx[4])
{
    struct spi_ioc_transfer transfer;

    /*
     * Protocol words are packed as bytes in control_protocol.c. Keep spidev at
     * 8 bits/word so byte order is explicit and portable across SPI drivers.
     */
    memset(&transfer, 0, sizeof(transfer));
    transfer.tx_buf = (unsigned long)tx;
    transfer.rx_buf = (unsigned long)rx;
    transfer.len = 4;
    transfer.speed_hz = spi->speed_hz;
    transfer.delay_usecs = 0;
    transfer.bits_per_word = 8;
    transfer.cs_change = 0;

    if (ioctl(spi->fd, SPI_IOC_MESSAGE(1), &transfer) < 1) {
        return -errno;
    }

    return 0;
}

static int rpi_spi_exchange(void *context,
                            MotorCommand command,
                            EncoderSample *sample)
{
    RpiSpiComm *spi = (RpiSpiComm *)context;
    uint8_t tx[4];
    uint8_t rx[4] = {0, 0, 0, 0};
    int result;

    protocol_pack_command(command, tx);

    result = spi_transfer(spi, tx, rx);
    if (result < 0) {
        return result;
    }

    if (sample != 0) {
        *sample = protocol_unpack_encoders(rx);
    }

    return 0;
}

static void rpi_spi_close_context(void *context)
{
    rpi_spi_comm_close((RpiSpiComm *)context);
}

int rpi_spi_comm_open(RpiSpiComm *spi,
                      MotorComm *comm,
                      unsigned channel,
                      unsigned speed_hz,
                      unsigned spi_flags)
{
    int fd;

    fd = spi_open_device(channel, speed_hz, spi_flags);
    if (fd < 0) {
        return fd;
    }

    spi->fd = fd;
    spi->speed_hz = speed_hz;
    spi->channel = channel;

    comm->context = spi;
    comm->exchange = rpi_spi_exchange;
    comm->close = rpi_spi_close_context;

    return 0;
}

void rpi_spi_comm_close(RpiSpiComm *spi)
{
    if (spi->fd >= 0) {
        close(spi->fd);
        spi->fd = -1;
    }
}
