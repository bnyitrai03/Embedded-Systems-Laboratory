CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
CPPFLAGS ?= -I. -Iapp/include -Icontrol/include -Icomm/include -Ivision/include -Icontroller/yaw_controller -Icontroller/pitch_controller -D_GNU_SOURCE -D_POSIX_C_SOURCE=200809L
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
VISION_SMOKE_TARGET = vision/test/vision_tracker_smoke

SRCS = \
	main.c \
	app/control_loop.c \
	vision/vision_tracker.c \
	vision/vision_blob.c \
	vision/vision_geometry.c \
	vision/vision_stream.c \
	comm/control_protocol.c \
	control/jiwy_calibration.c \
	control/twentysim_controller.c \
	app/homing.c \
	comm/rpi_spi_master.c \
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

.PHONY: all clean smoke-test vision-smoke mac-smoke

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(THREAD_FLAGS) -o $@ $(OBJS) $(LDLIBS) $(GST_LDLIBS) $(THREAD_FLAGS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(GST_CFLAGS) $(CFLAGS) $(THREAD_FLAGS) $(DEPFLAGS) -c -o $@ $<

smoke-test:
	$(CC) $(CFLAGS) -o test/spi_motor_smoke_test test/spi_motor_smoke_test.c

vision-smoke:
	$(CC) $(CPPFLAGS) $(GST_CFLAGS) $(CFLAGS) $(THREAD_FLAGS) \
		-o $(VISION_SMOKE_TARGET) \
		vision/test/vision_tracker_smoke.c \
		vision/vision_tracker.c vision/vision_blob.c \
		vision/vision_geometry.c vision/vision_stream.c \
		$(LDLIBS) $(GST_LDLIBS) $(THREAD_FLAGS)

mac-smoke:
	$(CC) $(CPPFLAGS) $(GST_CFLAGS) $(CFLAGS) $(THREAD_FLAGS) \
		-DJIWY_VISION_USE_AVFVIDEOSRC \
		-o $(VISION_SMOKE_TARGET) \
		vision/test/vision_tracker_smoke.c \
		vision/vision_tracker.c vision/vision_blob.c \
		vision/vision_geometry.c vision/vision_stream.c \
		$(LDLIBS) $(GST_LDLIBS) $(THREAD_FLAGS)

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS) test/spi_motor_smoke_test \
		$(VISION_SMOKE_TARGET)

-include $(DEPS)
