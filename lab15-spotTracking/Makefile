CC ?= cc
PKG_CONFIG ?= pkg-config
CFLAGS += -Wall -Wextra -std=c11

GST_PKGS = gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0
PREVIEW_PKGS = $(GST_PKGS) cairo

GST_CFLAGS = $(shell $(PKG_CONFIG) --cflags $(GST_PKGS))
GST_LDLIBS = $(shell $(PKG_CONFIG) --libs $(GST_PKGS))
PREVIEW_CFLAGS = $(shell $(PKG_CONFIG) --cflags $(PREVIEW_PKGS))
PREVIEW_LDLIBS = $(shell $(PKG_CONFIG) --libs $(PREVIEW_PKGS))

TARGETS = \
	green_ball_mac_preview \
	green_ball_mac_headless \
	green_ball_rpi_preview \
	green_ball_rpi_headless

.DEFAULT_GOAL := help

.PHONY: help mac-preview mac-headless rpi-preview rpi-headless clean

help:
	@printf "Pick one build target:\n"
	@printf "  make mac-preview     -> green_ball_mac_preview\n"
	@printf "  make mac-headless    -> green_ball_mac_headless\n"
	@printf "  make rpi-preview     -> green_ball_rpi_preview\n"
	@printf "  make rpi-headless    -> green_ball_rpi_headless\n"
	@printf "\n"
	@printf "Run with the default camera, or pass one camera target:\n"
	@printf "  ./green_ball_mac_preview 0\n"
	@printf "  ./green_ball_rpi_headless /dev/video0\n"

mac-preview: green_ball_mac_preview

mac-headless: green_ball_mac_headless

rpi-preview: green_ball_rpi_preview

rpi-headless: green_ball_rpi_headless

green_ball_mac_preview: green_ball_mac_preview.c
	$(CC) $(CFLAGS) $(PREVIEW_CFLAGS) $< -o $@ $(PREVIEW_LDLIBS)

green_ball_mac_headless: green_ball_mac_headless.c
	$(CC) $(CFLAGS) $(GST_CFLAGS) $< -o $@ $(GST_LDLIBS)

green_ball_rpi_preview: green_ball_rpi_preview.c
	$(CC) $(CFLAGS) $(PREVIEW_CFLAGS) $< -o $@ $(PREVIEW_LDLIBS)

green_ball_rpi_headless: green_ball_rpi_headless.c
	$(CC) $(CFLAGS) $(GST_CFLAGS) $< -o $@ $(GST_LDLIBS)

clean:
	rm -f $(TARGETS) green_ball_appsink
