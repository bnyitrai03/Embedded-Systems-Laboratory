CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
CPPFLAGS ?= -I. -Icontroller/yaw_controller -Icontroller/pitch_controller -D_GNU_SOURCE -D_POSIX_C_SOURCE=200809L
LDLIBS ?= -lm

TARGET = jiwy_controller

SRCS = \
	main.c \
	control_loop.c \
	control_protocol.c \
	jiwy_calibration.c \
	twentysim_controller.c \
	homing.c \
	rpi_spi_master.c \
	controller/yaw_controller/yaw_model.c \
	controller/yaw_controller/yaw_integ.c \
	controller/yaw_controller/yaw_funcs.c \
	controller/yaw_controller/yaw_submod.c \
	controller/pitch_controller/pitch_model.c \
	controller/pitch_controller/pitch_integ.c \
	controller/pitch_controller/pitch_funcs.c \
	controller/pitch_controller/pitch_submod.c

OBJS = $(SRCS:.c=.o)

.PHONY: all clean smoke-test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $(OBJS) $(LDLIBS)

smoke-test:
	$(CC) $(CFLAGS) -o test/spi_motor_smoke_test test/spi_motor_smoke_test.c

clean:
	rm -f $(TARGET) $(OBJS) test/spi_motor_smoke_test
