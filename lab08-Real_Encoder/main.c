#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#include "soc_system.h"

#define ENCODER_REG_YAW_DIR     0
#define ENCODER_REG_YAW_COUNT   1
#define ENCODER_REG_PITCH_DIR   2
#define ENCODER_REG_PITCH_COUNT 3

#define DEFAULT_SAMPLE_DELAY_US 100000

int main(int argc, char** argv) {
	uint32_t samples = 1;
	useconds_t sample_delay_us = DEFAULT_SAMPLE_DELAY_US;

	if (argc > 1) {
		samples = (uint32_t)strtoul(argv[1], NULL, 0);
	}
	if (argc > 2) {
		sample_delay_us = (useconds_t)strtoul(argv[2], NULL, 0);
	}

	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("Couldn't open /dev/mem\n");
		return -1;
	}

	volatile uint32_t* encoder_regs = (volatile uint32_t*)mmap(NULL,
		HPS_0_ARM_A9_0_ESL_BUS_DEMO_0_SPAN,
		PROT_READ,
		MAP_SHARED,
		fd,
		HPS_0_ARM_A9_0_ESL_BUS_DEMO_0_BASE);

	if (encoder_regs == MAP_FAILED) {
		perror("Couldn't map bridge.");
		close(fd);
		return -1;
	}

	uint32_t sample = 0;
	do {
		uint32_t yaw_dir = encoder_regs[ENCODER_REG_YAW_DIR] & 0x1u;
		int32_t yaw_count = (int32_t)encoder_regs[ENCODER_REG_YAW_COUNT];
		uint32_t pitch_dir = encoder_regs[ENCODER_REG_PITCH_DIR] & 0x1u;
		int32_t pitch_count = (int32_t)encoder_regs[ENCODER_REG_PITCH_COUNT];

		printf("Yaw: count=%ld direction=%u | Pitch: count=%ld direction=%u\n",
			(long)yaw_count,
			yaw_dir,
			(long)pitch_count,
			pitch_dir);

		sample++;
		if (samples == 0 || sample < samples) {
			usleep(sample_delay_us);
		}
	} while (samples == 0 || sample < samples);

	munmap((void*)encoder_regs, HPS_0_ARM_A9_0_ESL_BUS_DEMO_0_SPAN);
	close(fd);
	return 0;
}
