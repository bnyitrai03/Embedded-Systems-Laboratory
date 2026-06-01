CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
CPPFLAGS ?= -I. -Icontroller/yaw_controller -Icontroller/pitch_controller -D_GNU_SOURCE -D_POSIX_C_SOURCE=200809L
DEPFLAGS ?= -MMD -MP
LDLIBS ?= -lm
GST_PACKAGES = gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0 glib-2.0
GST_MISSING := $(shell pkg-config --exists $(GST_PACKAGES) || echo yes)
ifeq ($(GST_MISSING),yes)
$(error Missing GStreamer development packages. On Raspberry Pi OS install: sudo apt install pkg-config libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev)
endif
GST_CFLAGS := $(shell pkg-config --cflags $(GST_PACKAGES))
GST_LDLIBS := $(shell pkg-config --libs $(GST_PACKAGES))
THREAD_FLAGS ?= -pthread

TARGET = jiwy_controller

SRCS = \
	main.c \
	control_loop.c \
	vision_tracker.c \
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
DEPS = $(OBJS:.o=.d)

.PHONY: all clean smoke-test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(THREAD_FLAGS) -o $@ $(OBJS) $(LDLIBS) $(GST_LDLIBS) $(THREAD_FLAGS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(GST_CFLAGS) $(CFLAGS) $(THREAD_FLAGS) $(DEPFLAGS) -c -o $@ $<

smoke-test:
	$(CC) $(CFLAGS) -o test/spi_motor_smoke_test test/spi_motor_smoke_test.c

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS) test/spi_motor_smoke_test

-include $(DEPS)
