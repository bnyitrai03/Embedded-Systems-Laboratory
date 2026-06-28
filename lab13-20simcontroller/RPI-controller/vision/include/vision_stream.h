#ifndef VISION_STREAM_H
#define VISION_STREAM_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

#include "vision_blob.h"

typedef struct VisionTargetSnapshot VisionTargetSnapshot;

typedef struct {
    pthread_t thread;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    uint8_t *frame;
    size_t frame_size;
    uint64_t frame_sequence;
    int enabled;
    int port;
    int running;
    int thread_started;
    int listen_fd;
    int client_fd;
} VisionStream;

void vision_stream_init(VisionStream *stream);
int vision_stream_start(VisionStream *stream, int enabled, int port);
void vision_stream_stop(VisionStream *stream);
void vision_stream_publish_rgb(VisionStream *stream, const uint8_t *rgb, int width, int height, int stride, const VisionBlobDetection *detection, const VisionTargetSnapshot *snapshot);

#endif
