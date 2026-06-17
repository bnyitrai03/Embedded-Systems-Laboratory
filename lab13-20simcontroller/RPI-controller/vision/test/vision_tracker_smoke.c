#include "vision_tracker.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "jiwy_config.h"

static volatile sig_atomic_t keep_running = 1;

static void handle_signal(int signum)
{
    (void)signum;
    keep_running = 0;
}

static void print_usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [--camera DEVICE_OR_INDEX] [--no-stream] [--port PORT]\n",
            program);
}

int main(int argc, char **argv)
{
    const char *camera = NULL;
    int stream_enabled = JIWY_VISION_STREAM_ENABLED;
    int stream_port = JIWY_VISION_STREAM_PORT;
    VisionTracker tracker;
    unsigned sample = 0;
    int result;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--camera") == 0 && i + 1 < argc) {
            camera = argv[++i];
        } else if (strcmp(argv[i], "--no-stream") == 0) {
            stream_enabled = 0;
        } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            char *end = NULL;
            long parsed = strtol(argv[++i], &end, 10);

            if (end == argv[i] || *end != '\0' || parsed <= 0 ||
                parsed > 65535) {
                print_usage(argv[0]);
                return 2;
            }
            stream_port = (int)parsed;
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    vision_tracker_init(&tracker);
    result = vision_tracker_start(&tracker,
                                  camera,
                                  1,
                                  stream_enabled,
                                  stream_port);
    if (result < 0) {
        fprintf(stderr, "vision tracker start failed: %s\n", strerror(-result));
        return 1;
    }

    if (stream_enabled) {
        printf("Open http://127.0.0.1:%d/ for the annotated frame.\n",
               stream_port);
    }
    printf("Press Ctrl-C to stop.\n");

    while (keep_running) {
        VisionTargetSnapshot snapshot;

        result = vision_tracker_read_latest(&tracker, &snapshot);
        if (result == 0) {
            if (snapshot.valid) {
                printf("sample=%u DETECTED yaw=%.4f pitch=%.4f "
                       "dt=%.2fms proc=%.0fus late=%d frame=%llu\n",
                       sample,
                       snapshot.yaw_error_rad,
                       snapshot.pitch_error_rad,
                       snapshot.frame_interval_ms,
                       snapshot.process_us,
                       snapshot.late_frame,
                       (unsigned long long)snapshot.frame_count);
            } else {
                printf("sample=%u NO_TARGET dt=%.2fms proc=%.0fus "
                       "late=%d frame=%llu\n",
                       sample,
                       snapshot.frame_interval_ms,
                       snapshot.process_us,
                       snapshot.late_frame,
                       (unsigned long long)snapshot.frame_count);
            }
        }

        ++sample;
        sleep(1);
    }

    vision_tracker_stop(&tracker);
    return 0;
}
