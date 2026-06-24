#include "vision_blob.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "jiwy_config.h"

#define VISION_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VISION_MAX(a, b) ((a) > (b) ? (a) : (b))

void vision_blob_tracker_init(VisionBlobTracker *tracker)
{
    if (tracker == NULL) {
        return;
    }

    memset(tracker, 0, sizeof(*tracker));
}

void vision_blob_tracker_destroy(VisionBlobTracker *tracker)
{
    if (tracker == NULL) {
        return;
    }

    free(tracker->green_mask);
    free(tracker->component_queue);
    tracker->green_mask = NULL;
    tracker->component_queue = NULL;
    tracker->detection_capacity = 0;
}

static int is_green_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t max_value = VISION_MAX(r, VISION_MAX(g, b));
    uint8_t min_value = VISION_MIN(r, VISION_MIN(g, b));
    int delta = (int)max_value - (int)min_value;
    int hue;
    int whiteness_percent;
    int blackness_percent;

    if (max_value < JIWY_VISION_GREEN_MIN_CHANNEL ||
        delta < JIWY_VISION_GREEN_MIN_DELTA ||
        g != max_value) {
        return 0;
    }

    hue = 120 + (60 * ((int)b - (int)r)) / delta;
    whiteness_percent = ((int)min_value * 100) / 255;
    blackness_percent = ((255 - (int)max_value) * 100) / 255;

    return hue >= HUE_LOWER_LIMIT &&
           hue <= HUE_UPPER_LIMIT &&
           whiteness_percent >= JIWY_VISION_GREEN_MIN_WHITENESS_PERCENT &&
           whiteness_percent <= JIWY_VISION_GREEN_MAX_WHITENESS_PERCENT &&
           blackness_percent >= JIWY_VISION_GREEN_MIN_BLACKNESS_PERCENT &&
           blackness_percent <= JIWY_VISION_GREEN_MAX_BLACKNESS_PERCENT;
}

static int ensure_detection_buffers(VisionBlobTracker *tracker,
                                    size_t pixel_count)
{
    uint8_t *new_mask;
    int *new_queue;

    if (pixel_count > (size_t)INT_MAX) {
        return -EOVERFLOW;
    }
    if (tracker->detection_capacity >= pixel_count) {
        return 0;
    }

    new_mask = (uint8_t *)malloc(pixel_count);
    new_queue = (int *)malloc(pixel_count * sizeof(*new_queue));
    if (new_mask == NULL || new_queue == NULL) {
        free(new_mask);
        free(new_queue);
        return -ENOMEM;
    }

    free(tracker->green_mask);
    free(tracker->component_queue);
    tracker->green_mask = new_mask;
    tracker->component_queue = new_queue;
    tracker->detection_capacity = pixel_count;

    return 0;
}

static int blob_shape_is_valid(uint64_t pixels,
                               int min_x,
                               int min_y,
                               int max_x,
                               int max_y)
{
    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;
    int min_side = VISION_MIN(width, height);
    int max_side = VISION_MAX(width, height);
    uint64_t box_area = (uint64_t)width * (uint64_t)height;

    if (pixels < JIWY_VISION_MIN_GREEN_PIXELS ||
        width < JIWY_VISION_BLOB_MIN_WIDTH ||
        height < JIWY_VISION_BLOB_MIN_HEIGHT ||
        min_side <= 0) {
        return 0;
    }

    if (max_side * 100 > min_side * JIWY_VISION_BLOB_MAX_ASPECT_PERCENT) {
        return 0;
    }

    return pixels * 100 >=
        box_area * (uint64_t)JIWY_VISION_BLOB_MIN_FILL_PERCENT;
}

static void find_green_blob(VisionBlobTracker *tracker,
                            const uint8_t *rgb,
                            int width,
                            int height,
                            int stride,
                            VisionBlobDetection *detection)
{
    int tracking_locked =
        tracker->have_filtered_object &&
        tracker->lost_frames < JIWY_VISION_TRACK_RESET_LOST_FRAMES;
    double best_score = -HUGE_VAL;

    memset(detection, 0, sizeof(*detection));

    for (int y = 0; y < height; ++y) {
        const uint8_t *row = rgb + y * stride;
        size_t row_offset = (size_t)y * (size_t)width;

        for (int x = 0; x < width; ++x) {
            const uint8_t *pixel = row + x * 3;
            size_t index = row_offset + (size_t)x;

            if (is_green_pixel(pixel[0], pixel[1], pixel[2])) {
                tracker->green_mask[index] = 1;
                ++detection->total_green_pixels;
            } else {
                tracker->green_mask[index] = 0;
            }
        }
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int start_index = y * width + x;
            int head = 0;
            int tail = 0;
            uint64_t pixels = 0;
            uint64_t sum_x = 0;
            uint64_t sum_y = 0;
            int min_x = width;
            int min_y = height;
            int max_x = 0;
            int max_y = 0;
            double object_x;
            double object_y;
            double score;

            if (tracker->green_mask[start_index] == 0) {
                continue;
            }

            tracker->green_mask[start_index] = 0;
            tracker->component_queue[tail++] = start_index;

            while (head < tail) {
                int pixel_index = tracker->component_queue[head++];
                int pixel_x = pixel_index % width;
                int pixel_y = pixel_index / width;

                ++pixels;
                sum_x += (uint64_t)pixel_x;
                sum_y += (uint64_t)pixel_y;
                min_x = VISION_MIN(min_x, pixel_x);
                min_y = VISION_MIN(min_y, pixel_y);
                max_x = VISION_MAX(max_x, pixel_x);
                max_y = VISION_MAX(max_y, pixel_y);

                if (pixel_x > 0 &&
                    tracker->green_mask[pixel_index - 1] != 0) {
                    tracker->green_mask[pixel_index - 1] = 0;
                    tracker->component_queue[tail++] = pixel_index - 1;
                }
                if (pixel_x + 1 < width &&
                    tracker->green_mask[pixel_index + 1] != 0) {
                    tracker->green_mask[pixel_index + 1] = 0;
                    tracker->component_queue[tail++] = pixel_index + 1;
                }
                if (pixel_y > 0 &&
                    tracker->green_mask[pixel_index - width] != 0) {
                    tracker->green_mask[pixel_index - width] = 0;
                    tracker->component_queue[tail++] = pixel_index - width;
                }
                if (pixel_y + 1 < height &&
                    tracker->green_mask[pixel_index + width] != 0) {
                    tracker->green_mask[pixel_index + width] = 0;
                    tracker->component_queue[tail++] = pixel_index + width;
                }
            }

            if (!blob_shape_is_valid(pixels, min_x, min_y, max_x, max_y)) {
                continue;
            }

            object_x = (double)sum_x / (double)pixels;
            object_y = (double)sum_y / (double)pixels;
            score = (double)pixels;

            if (tracking_locked) {
                double dx = object_x - tracker->filtered_object_x;
                double dy = object_y - tracker->filtered_object_y;
                double distance = sqrt(dx * dx + dy * dy);

                if (distance > JIWY_VISION_TRACK_MAX_JUMP_PIXELS) {
                    continue;
                }

                score -= distance * distance * 0.25;
            }

            if (!detection->valid || score > best_score) {
                detection->valid = 1;
                detection->blob_pixels = pixels;
                detection->min_x = min_x;
                detection->min_y = min_y;
                detection->max_x = max_x;
                detection->max_y = max_y;
                detection->object_x = object_x;
                detection->object_y = object_y;
                best_score = score;
            }
        }
    }
}

static void smooth_blob_detection(VisionBlobTracker *tracker,
                                  VisionBlobDetection *detection)
{
    double alpha = JIWY_VISION_TRACK_SMOOTHING_ALPHA;

    if (!detection->valid) {
        ++tracker->lost_frames;
        if (tracker->lost_frames >= JIWY_VISION_TRACK_RESET_LOST_FRAMES) {
            tracker->have_filtered_object = 0;
        }
        return;
    }

    if (alpha < 0.0) {
        alpha = 0.0;
    } else if (alpha > 1.0) {
        alpha = 1.0;
    }

    if (!tracker->have_filtered_object) {
        tracker->filtered_object_x = detection->object_x;
        tracker->filtered_object_y = detection->object_y;
        tracker->have_filtered_object = 1;
    } else {
        tracker->filtered_object_x +=
            alpha * (detection->object_x - tracker->filtered_object_x);
        tracker->filtered_object_y +=
            alpha * (detection->object_y - tracker->filtered_object_y);
    }

    detection->object_x = tracker->filtered_object_x;
    detection->object_y = tracker->filtered_object_y;
    tracker->lost_frames = 0;
}

int vision_blob_tracker_process_rgb(VisionBlobTracker *tracker,
                                    const uint8_t *rgb,
                                    int width,
                                    int height,
                                    int stride,
                                    VisionBlobDetection *detection)
{
    size_t pixel_count;
    int result;

    if (tracker == NULL || rgb == NULL || detection == NULL ||
        width <= 0 || height <= 0 || stride < width * 3) {
        return -EINVAL;
    }
    if ((size_t)height > (size_t)INT_MAX / (size_t)width) {
        return -EOVERFLOW;
    }

    pixel_count = (size_t)width * (size_t)height;
    result = ensure_detection_buffers(tracker, pixel_count);
    if (result < 0) {
        memset(detection, 0, sizeof(*detection));
        return result;
    }

    find_green_blob(tracker, rgb, width, height, stride, detection);
    smooth_blob_detection(tracker, detection);
    return 0;
}
