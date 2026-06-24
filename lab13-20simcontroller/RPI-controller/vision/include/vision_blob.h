#ifndef VISION_BLOB_H
#define VISION_BLOB_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int valid;
    uint64_t total_green_pixels;
    uint64_t blob_pixels;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    double object_x;
    double object_y;
} VisionBlobDetection;

typedef struct {
    uint8_t *green_mask;
    int *component_queue;
    size_t detection_capacity;
    double filtered_object_x;
    double filtered_object_y;
    int have_filtered_object;
    unsigned lost_frames;
} VisionBlobTracker;

void vision_blob_tracker_init(VisionBlobTracker *tracker);
void vision_blob_tracker_destroy(VisionBlobTracker *tracker);
int vision_blob_tracker_process_rgb(VisionBlobTracker *tracker,
                                    const uint8_t *rgb,
                                    int width,
                                    int height,
                                    int stride,
                                    VisionBlobDetection *detection);

#endif
