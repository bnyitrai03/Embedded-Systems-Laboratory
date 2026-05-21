#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

// Usage: ./spi_master [speed_hz] [byte 0-255]
// Expected behavior: The Master (RPI4) sends a byte parsed from terminal to the SPI slave (IcoBoard) and expects to receive the byte incremented by 1 in response.

#define DEFAULT_SPEED 100000
#define CHANNEL 1
#define MAX_SPI_BUFSIZ 8192

int spiOpen(unsigned spiChan, unsigned spiBaud, unsigned spiFlags) {
    int fd;
    uint8_t spiMode;
    uint8_t spiBits = 8;
    char dev[32];

    spiMode = spiFlags & 0x3;
    snprintf(dev, sizeof(dev), "/dev/spidev0.%u", spiChan);

    fd = open(dev, O_RDWR);
    if (fd < 0) return -1;

    if (ioctl(fd, SPI_IOC_WR_MODE, &spiMode) < 0) {
        close(fd);
        return -2;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spiBits) < 0) {
        close(fd);
        return -3;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spiBaud) < 0) {
        close(fd);
        return -4;
    }

    return fd;
}

int spiClose(int fd) {
    return close(fd);
}

int spiXfer(int fd, unsigned speed, uint8_t *txBuf, uint8_t *rxBuf, unsigned count) {
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long)txBuf;
    spi.rx_buf = (unsigned long)rxBuf;
    spi.len = count;
    spi.speed_hz = speed;
    spi.delay_usecs = 0;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    return ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
}

int main(int argc, char *argv[]) {
    int fd;
    unsigned spiChan = CHANNEL;
    unsigned speed = DEFAULT_SPEED;
    uint8_t input;
    uint8_t tx;
    uint8_t rx;
    uint8_t expected;

    if (argc < 2) {
        fprintf(stderr, "Usage: [speed_hz] [byte 0-255]\n");
        return 1;
    }

    speed = (unsigned)atoi(argv[1]);
    if (speed < 32000 || speed > 250000000) {
        speed = DEFAULT_SPEED;
    }

    input = (uint8_t)atoi(argv[2]);
    if (input > 255) {
        fprintf(stderr, "Input byte must be between 0 and 255\n");
        return 1;
    }

    fd = spiOpen(spiChan, speed, 0);   // mode 0
    if (fd < 0) {
        fprintf(stderr, "Failed to open SPI device /dev/spidev0.%u\n", spiChan);
        return 1;
    }

    tx = input;
    rx = 0x00;

    if (spiXfer(fd, speed, &tx, &rx, sizeof(tx)) < 1) {
        fprintf(stderr, "SPI transfer failed\n");
        spiClose(fd);
        return 1;
    }
    
    printf("TX = %u (0x%02X)\n", tx, tx);
    printf("RX = %u (0x%02X)\n", rx, rx);

    spiClose(fd);
    return 0;
}