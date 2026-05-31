#include "soc_system.h"

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

static uint16_t clamp_pwm_u14(uint16_t pwm)
{
    return (pwm > MOTOR_FIELD_PWM_MASK) ? MOTOR_FIELD_PWM_MASK : pwm;
}

static uint32_t pack_command_word(MotorCommand command)
{
    uint16_t yaw_pwm   = clamp_pwm_u14(command.yaw.pwm);
    uint16_t pitch_pwm = clamp_pwm_u14(command.pitch.pwm);

    return PACK_MOTOR32(
        command.yaw.direction ? 1u : 0u,
        command.yaw.enable ? 1u : 0u,
        yaw_pwm,
        command.pitch.direction ? 1u : 0u,
        command.pitch.enable ? 1u : 0u,
        pitch_pwm
    );
}

static EncoderSample unpack_encoder_word(uint32_t word)
{
    EncoderSample sample;
    sample.pitch = (int16_t)(word & 0xFFFFu);
    sample.yaw   = (int16_t)((word >> 16) & 0xFFFFu);
    return sample;
}

static int soc_fpga_exchange(void *context,
                             MotorCommand command,
                             EncoderSample *sample)
{
    SocFpgaComm *soc = (SocFpgaComm *)context;
    uint32_t command_word;
    uint32_t encoder_word;

    if (soc == NULL || soc->regs == NULL) {
        return -ENODEV;
    }

    command_word = pack_command_word(command);
    soc->regs[MIDDLEWARE_REG_MOTORS] = command_word;

    encoder_word = soc->regs[MIDDLEWARE_REG_ENCODERS];

    if (sample != NULL) {
        *sample = unpack_encoder_word(encoder_word);
    }

    return 0;
}

static void soc_fpga_close_context(void *context)
{
    soc_fpga_comm_close((SocFpgaComm *)context);
}

int soc_fpga_comm_open(SocFpgaComm *soc, MotorComm *comm)
{
    void *mapped;

    if (soc == NULL || comm == NULL) {
        return -EINVAL;
    }

    soc->mem_fd = -1;
    soc->regs = NULL;
    soc->phys_base = SOC_FPGA_DEFAULT_BASE;
    soc->map_span = SOC_FPGA_DEFAULT_SPAN;

    soc->mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (soc->mem_fd < 0) {
        return -errno;
    }

    mapped = mmap(NULL,
                  soc->map_span,
                  PROT_READ | PROT_WRITE,
                  MAP_SHARED,
                  soc->mem_fd,
                  soc->phys_base);
    if (mapped == MAP_FAILED) {
        int saved_errno = errno;
        close(soc->mem_fd);
        soc->mem_fd = -1;
        return -saved_errno;
    }

    soc->regs = (volatile uint32_t *)mapped;

    comm->context = soc;
    comm->exchange = soc_fpga_exchange;
    comm->close = soc_fpga_close_context;

    return 0;
}

void soc_fpga_comm_close(SocFpgaComm *soc)
{
    if (soc == NULL) {
        return;
    }

    if (soc->regs != NULL) {
        munmap((void *)soc->regs, soc->map_span);
        soc->regs = NULL;
    }

    if (soc->mem_fd >= 0) {
        close(soc->mem_fd);
        soc->mem_fd = -1;
    }
}