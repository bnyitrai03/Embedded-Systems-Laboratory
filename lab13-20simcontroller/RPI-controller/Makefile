CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
CPPFLAGS ?= -I. -D_POSIX_C_SOURCE=200809L
LDLIBS ?= -lm

TARGET = jiwy_controller

SRCS = \
	main.c \
	control_loop.c \
	control_protocol.c \
	controller_adapter.c \
	homing.c \
	rpi_spi_master.c \
	controller/pitch_controller.c \
	controller/yaw_controller.c \
	controller/xxfuncs.c

OBJS = $(SRCS:.c=.o)

.PHONY: all clean smoke-test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $(OBJS) $(LDLIBS)

smoke-test:
	$(CC) $(CFLAGS) -o test/spi_motor_smoke_test test/spi_motor_smoke_test.c

clean:
	rm -f $(TARGET) $(OBJS) test/spi_motor_smoke_test
