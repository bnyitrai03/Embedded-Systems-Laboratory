/*
 * Standalone Raspberry Pi SPI smoke test for the FPGA motor interface.
 *
 * Purpose:
 *   Send one small yaw/pitch PWM command frame repeatedly and print the
 *   returned encoder counts. This is meant to verify the FPGA SPI protocol
 *   before integrating the full controller application.
 *
 * Build on Raspberry Pi:
 *   gcc -O2 -Wall -Wextra -std=c11 -o spi_motor_smoke_test spi_motor_smoke_test.c
 *
 * Example, yaw only, low PWM, positive direction:
 *   ./spi_motor_smoke_test 100000 1500 1 0 0 200 10
 *
 * Arguments:
 *   speed_hz yaw_pwm yaw_dir pitch_pwm pitch_dir loops period_ms
 *
 * Frame format, big-endian:
 *   TX byte 0..1: yaw   bit15 direction, bit14 enable, bit13..0 PWM
 *   TX byte 2..3: pitch bit15 direction, bit14 enable, bit13..0 PWM
 *   RX byte 0..1: yaw signed int16 encoder count
 *   RX byte 2..3: pitch signed int16 encoder count
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_SPI_DEVICE "/dev/spidev0.1"
#define DEFAULT_SPEED_HZ 100000u
#define DEFAULT_LOOPS 200u
#define DEFAULT_PERIOD_MS 10u
#define PWM_MAX 2500u

static volatile sig_atomic_t keep_running = 1;

typedef struct {
    int16_t yaw;
    int16_t pitch;
} EncoderSample;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    keep_running = 0;
}

static unsigned parse_unsigned(const char *text, unsigned fallback)
{
    char *end = 0;
    unsigned long value;

    if (text == 0) {
        return fallback;
    }

    errno = 0;
    value = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value > 0xFFFFFFFFul) {
        return fallback;
    }

    return (unsigned)value;
}

static void sleep_ms(unsigned ms)
{
    struct timespec request;

    request.tv_sec = (time_t)(ms / 1000u);
    request.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&request, 0);
}

static uint16_t pack_axis(unsigned pwm, unsigned direction)
{
    unsigned enable = pwm > 0u;

    if (pwm > PWM_MAX) {
        pwm = PWM_MAX;
    }

    return (uint16_t)(((direction ? 1u : 0u) << 15) |
                      ((enable ? 1u : 0u) << 14) |
                      pwm);
}

static void pack_frame(uint8_t tx[4],
                       unsigned yaw_pwm,
                       unsigned yaw_direction,
                       unsigned pitch_pwm,
                       unsigned pitch_direction)
{
    uint16_t yaw = pack_axis(yaw_pwm, yaw_direction);
    uint16_t pitch = pack_axis(pitch_pwm, pitch_direction);

    tx[0] = (uint8_t)(yaw >> 8);
    tx[1] = (uint8_t)(yaw & 0xFFu);
    tx[2] = (uint8_t)(pitch >> 8);
    tx[3] = (uint8_t)(pitch & 0xFFu);
}

static EncoderSample unpack_frame(const uint8_t rx[4])
{
    EncoderSample sample;
    uint16_t yaw = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    uint16_t pitch = (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);

    sample.yaw = (int16_t)yaw;
    sample.pitch = (int16_t)pitch;
    return sample;
}

static int spi_open(const char *device, unsigned speed_hz)
{
    int fd;
    uint8_t mode = SPI_MODE_0;

    fd = open(device, O_RDWR);
    if (fd < 0) {
        return -errno;
    }

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
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

static int spi_exchange(int fd, unsigned speed_hz, const uint8_t tx[4], uint8_t rx[4])
{
    struct spi_ioc_transfer transfer;
    uint8_t local_tx[4] = {tx[0], tx[1], tx[2], tx[3]};

    memset(&transfer, 0, sizeof(transfer));
    transfer.tx_buf = (unsigned long)local_tx;
    transfer.rx_buf = (unsigned long)rx;
    transfer.len = 4;
    transfer.speed_hz = speed_hz;
    /*
     * Leave bits_per_word as 0 so spidev uses the device default, normally
     * 8 bits. Some Raspberry Pi kernels reject SPI_IOC_WR_BITS_PER_WORD even
     * for 8-bit transfers.
     */
    transfer.bits_per_word = 0;
    transfer.delay_usecs = 0;
    transfer.cs_change = 0;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &transfer) < 1) {
        return -errno;
    }

    return 0;
}

static int send_stop(int fd, unsigned speed_hz)
{
    uint8_t tx[4] = {0, 0, 0, 0};
    uint8_t rx[4] = {0, 0, 0, 0};

    return spi_exchange(fd, speed_hz, tx, rx);
}

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [speed_hz] [yaw_pwm] [yaw_dir] [pitch_pwm] [pitch_dir] [loops] [period_ms]\n"
            "Defaults: speed=%u, yaw_pwm=0, yaw_dir=1, pitch_pwm=0, pitch_dir=1, loops=%u, period_ms=%u\n"
            "Directions are raw FPGA direction bits: 0 or 1.\n",
            program,
            DEFAULT_SPEED_HZ,
            DEFAULT_LOOPS,
            DEFAULT_PERIOD_MS);
}

int main(int argc, char *argv[])
{
    unsigned speed_hz = DEFAULT_SPEED_HZ;
    unsigned yaw_pwm = 0;
    unsigned yaw_direction = 1;
    unsigned pitch_pwm = 0;
    unsigned pitch_direction = 1;
    unsigned loops = DEFAULT_LOOPS;
    unsigned period_ms = DEFAULT_PERIOD_MS;
    uint8_t tx[4];
    uint8_t rx[4];
    int fd;
    int result;
    unsigned i;

    if (argc > 8) {
        print_usage(argv[0]);
        return 2;
    }

    speed_hz = parse_unsigned(argc > 1 ? argv[1] : 0, speed_hz);
    yaw_pwm = parse_unsigned(argc > 2 ? argv[2] : 0, yaw_pwm);
    yaw_direction = parse_unsigned(argc > 3 ? argv[3] : 0, yaw_direction) ? 1u : 0u;
    pitch_pwm = parse_unsigned(argc > 4 ? argv[4] : 0, pitch_pwm);
    pitch_direction = parse_unsigned(argc > 5 ? argv[5] : 0, pitch_direction) ? 1u : 0u;
    loops = parse_unsigned(argc > 6 ? argv[6] : 0, loops);
    period_ms = parse_unsigned(argc > 7 ? argv[7] : 0, period_ms);

    if (yaw_pwm > PWM_MAX || pitch_pwm > PWM_MAX) {
        fprintf(stderr, "PWM values must be <= %u\n", PWM_MAX);
        return 2;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    fd = spi_open(DEFAULT_SPI_DEVICE, speed_hz);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", DEFAULT_SPI_DEVICE, strerror(-fd));
        return 1;
    }

    pack_frame(tx, yaw_pwm, yaw_direction, pitch_pwm, pitch_direction);

    printf("SPI device: %s, speed: %u Hz\n", DEFAULT_SPI_DEVICE, speed_hz);
    printf("TX bytes: %02X %02X %02X %02X\n", tx[0], tx[1], tx[2], tx[3]);
    printf("yaw_pwm=%u yaw_dir=%u pitch_pwm=%u pitch_dir=%u loops=%u period_ms=%u\n",
           yaw_pwm,
           yaw_direction,
           pitch_pwm,
           pitch_direction,
           loops,
           period_ms);
    printf("Press Ctrl-C to stop early; a stop frame is sent before exit.\n");

    for (i = 0; i < loops && keep_running; ++i) {
        EncoderSample sample;

        memset(rx, 0, sizeof(rx));
        result = spi_exchange(fd, speed_hz, tx, rx);
        if (result < 0) {
            fprintf(stderr, "SPI transfer failed: %s\n", strerror(-result));
            send_stop(fd, speed_hz);
            close(fd);
            return 1;
        }

        sample = unpack_frame(rx);
        printf("%5u rx=%02X %02X %02X %02X yaw=%6d pitch=%6d\n",
               i,
               rx[0],
               rx[1],
               rx[2],
               rx[3],
               sample.yaw,
               sample.pitch);

        sleep_ms(period_ms);
    }

    send_stop(fd, speed_hz);
    close(fd);
    printf("Stopped motors and closed SPI\n");
    return 0;
}
