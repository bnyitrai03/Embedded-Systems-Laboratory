#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include "soc_system.h"

#define PWM_CONTROL_ENABLE 0x1u
#define PWM_CONTROL_DIR    0x2u

#define PWM_REG_CONTROL 0
#define PWM_REG_DUTY    1

int main(int argc, char** argv) {
	uint32_t duty = 1250;
	uint32_t direction = 0;
	uint32_t enable = 1;

	if (argc > 1) {
		duty = (uint32_t)strtoul(argv[1], NULL, 0);
	}
	if (argc > 2) {
		direction = (uint32_t)strtoul(argv[2], NULL, 0);
	}
	if (argc > 3) {
		enable = (uint32_t)strtoul(argv[3], NULL, 0);
	}

	if (duty > 2500) {
		fprintf(stderr, "Duty must be between 0 and 2500 for 20 kHz PWM.\n");
		return -1;
	}

	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("Couldn't open /dev/mem\n");
		return -1;
	}

	volatile uint32_t* pwm_regs = (volatile uint32_t*)mmap(NULL,
		PWM_WRAPPER_0_SPAN,
		PROT_READ | PROT_WRITE,
		MAP_SHARED,
		fd,
		PWM_WRAPPER_0_BASE);

	if (pwm_regs == MAP_FAILED) {
		perror("Couldn't map bridge.");
		close(fd);
		return -1;
	}

	uint32_t control = 0;
	if (enable) {
		control |= PWM_CONTROL_ENABLE;
	}
	if (direction) {
		control |= PWM_CONTROL_DIR;
	}

	pwm_regs[PWM_REG_DUTY] = duty;
	pwm_regs[PWM_REG_CONTROL] = control;

	printf("PWM: enable=%u direction=%u duty=%u\n",
		enable ? 1u : 0u,
		direction ? 1u : 0u,
		duty);

	munmap((void*)pwm_regs, PWM_WRAPPER_0_SPAN);
	close(fd);
	return 0;
}
