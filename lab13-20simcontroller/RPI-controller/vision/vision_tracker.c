#include "vision_tracker.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include "jiwy_config.h"

typedef struct {
    VisionTracker *tracker;
    uint64_t frame_count;
    struct timespec last_frame_time;
    int have_last_frame_time;
    guint8 *green_mask;
    int *component_queue;
    size_t detection_capacity;
    double filtered_object_x;
    double filtered_object_y;
    int have_filtered_object;
    unsigned lost_frames;
} VisionPipelineData;

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

static long timespec_diff_us(struct timespec end, struct timespec start)
{
    time_t sec = end.tv_sec - start.tv_sec;
    long nsec = end.tv_nsec - start.tv_nsec;

    return (long)sec * 1000000L + nsec / 1000L;
}

static gboolean is_green_pixel(guint8 r, guint8 g, guint8 b)
{
    guint8 max_value = MAX(r, MAX(g, b));
    guint8 min_value = MIN(r, MIN(g, b));
    int delta = max_value - min_value;

    int hue;
    int whiteness_percent;
    int blackness_percent;

    if (max_value < JIWY_VISION_GREEN_MIN_CHANNEL ||
        delta < JIWY_VISION_GREEN_MIN_DELTA ||
        g != max_value) {
        return FALSE;
    }

    hue = 120 + (60 * ((int)b - (int)r)) / delta;

    whiteness_percent = (min_value * 100) / 255;
    blackness_percent = ((255 - max_value) * 100) / 255;

    return hue >= HUE_LOWER_LIMIT && hue <= HUE_UPPER_LIMIT &&
           whiteness_percent >= JIWY_VISION_GREEN_MIN_WHITENESS_PERCENT &&
           whiteness_percent <= JIWY_VISION_GREEN_MAX_WHITENESS_PERCENT &&
           blackness_percent >= JIWY_VISION_GREEN_MIN_BLACKNESS_PERCENT &&
           blackness_percent <= JIWY_VISION_GREEN_MAX_BLACKNESS_PERCENT;
}

static int ensure_detection_buffers(VisionPipelineData *data,
                                    size_t pixel_count)
{
    guint8 *new_mask;
    int *new_queue;

    if (pixel_count > (size_t)INT_MAX) {
        return -EOVERFLOW;
    }
    if (data->detection_capacity >= pixel_count) {
        return 0;
    }

    new_mask = (guint8 *)malloc(pixel_count);
    new_queue = (int *)malloc(pixel_count * sizeof(*new_queue));
    if (new_mask == NULL || new_queue == NULL) {
        free(new_mask);
        free(new_queue);
        return -ENOMEM;
    }

    free(data->green_mask);
    free(data->component_queue);
    data->green_mask = new_mask;
    data->component_queue = new_queue;
    data->detection_capacity = pixel_count;

    return 0;
}

static gboolean blob_shape_is_valid(uint64_t pixels,
                                    int min_x,
                                    int min_y,
                                    int max_x,
                                    int max_y)
{
    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;
    int min_side = MIN(width, height);
    int max_side = MAX(width, height);
    uint64_t box_area = (uint64_t)width * (uint64_t)height;

    if (pixels < JIWY_VISION_MIN_GREEN_PIXELS ||
        width < JIWY_VISION_BLOB_MIN_WIDTH ||
        height < JIWY_VISION_BLOB_MIN_HEIGHT ||
        min_side <= 0) {
        return FALSE;
    }

    if (max_side * 100 > min_side * JIWY_VISION_BLOB_MAX_ASPECT_PERCENT) {
        return FALSE;
    }

    return pixels * 100 >=
        box_area * (uint64_t)JIWY_VISION_BLOB_MIN_FILL_PERCENT;
}

static void find_green_blob(VisionPipelineData *data,
                            const guint8 *rgb,
                            int width,
                            int height,
                            int stride,
                            VisionBlobDetection *detection)
{
    gboolean tracking_locked =
        data->have_filtered_object &&
        data->lost_frames < JIWY_VISION_TRACK_RESET_LOST_FRAMES;
    double best_score = -HUGE_VAL;

    memset(detection, 0, sizeof(*detection));

    for (int y = 0; y < height; ++y) {
        const guint8 *row = rgb + y * stride;
        size_t row_offset = (size_t)y * (size_t)width;

        for (int x = 0; x < width; ++x) {
            const guint8 *pixel = row + x * 3;
            size_t index = row_offset + (size_t)x;

            if (is_green_pixel(pixel[0], pixel[1], pixel[2])) {
                data->green_mask[index] = 1;
                ++detection->total_green_pixels;
            } else {
                data->green_mask[index] = 0;
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

            if (data->green_mask[start_index] == 0) {
                continue;
            }

            data->green_mask[start_index] = 0;
            data->component_queue[tail++] = start_index;

            while (head < tail) {
                int pixel_index = data->component_queue[head++];
                int pixel_x = pixel_index % width;
                int pixel_y = pixel_index / width;

                ++pixels;
                sum_x += (uint64_t)pixel_x;
                sum_y += (uint64_t)pixel_y;
                min_x = MIN(min_x, pixel_x);
                min_y = MIN(min_y, pixel_y);
                max_x = MAX(max_x, pixel_x);
                max_y = MAX(max_y, pixel_y);

                if (pixel_x > 0 &&
                    data->green_mask[pixel_index - 1] != 0) {
                    data->green_mask[pixel_index - 1] = 0;
                    data->component_queue[tail++] = pixel_index - 1;
                }
                if (pixel_x + 1 < width &&
                    data->green_mask[pixel_index + 1] != 0) {
                    data->green_mask[pixel_index + 1] = 0;
                    data->component_queue[tail++] = pixel_index + 1;
                }
                if (pixel_y > 0 &&
                    data->green_mask[pixel_index - width] != 0) {
                    data->green_mask[pixel_index - width] = 0;
                    data->component_queue[tail++] = pixel_index - width;
                }
                if (pixel_y + 1 < height &&
                    data->green_mask[pixel_index + width] != 0) {
                    data->green_mask[pixel_index + width] = 0;
                    data->component_queue[tail++] = pixel_index + width;
                }
            }

            if (!blob_shape_is_valid(pixels, min_x, min_y, max_x, max_y)) {
                continue;
            }

            object_x = (double)sum_x / (double)pixels;
            object_y = (double)sum_y / (double)pixels;
            score = (double)pixels;

            if (tracking_locked) {
                double dx = object_x - data->filtered_object_x;
                double dy = object_y - data->filtered_object_y;
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

static void smooth_blob_detection(VisionPipelineData *data,
                                  VisionBlobDetection *detection)
{
    double alpha = JIWY_VISION_TRACK_SMOOTHING_ALPHA;

    if (!detection->valid) {
        ++data->lost_frames;
        if (data->lost_frames >= JIWY_VISION_TRACK_RESET_LOST_FRAMES) {
            data->have_filtered_object = 0;
        }
        return;
    }

    if (alpha < 0.0) {
        alpha = 0.0;
    } else if (alpha > 1.0) {
        alpha = 1.0;
    }

    if (!data->have_filtered_object) {
        data->filtered_object_x = detection->object_x;
        data->filtered_object_y = detection->object_y;
        data->have_filtered_object = 1;
    } else {
        data->filtered_object_x +=
            alpha * (detection->object_x - data->filtered_object_x);
        data->filtered_object_y +=
            alpha * (detection->object_y - data->filtered_object_y);
    }

    detection->object_x = data->filtered_object_x;
    detection->object_y = data->filtered_object_y;
    data->lost_frames = 0;
}


static void write_le16(guint8 *out, uint16_t value)
{
    out[0] = (guint8)(value & 0xFFu);
    out[1] = (guint8)((value >> 8) & 0xFFu);
}

static void write_le32(guint8 *out, uint32_t value)
{
    out[0] = (guint8)(value & 0xFFu);
    out[1] = (guint8)((value >> 8) & 0xFFu);
    out[2] = (guint8)((value >> 16) & 0xFFu);
    out[3] = (guint8)((value >> 24) & 0xFFu);
}

static int send_all(int fd, const void *buffer, size_t size)
{
    const guint8 *bytes = (const guint8 *)buffer;
#ifdef MSG_NOSIGNAL
    const int send_flags = MSG_NOSIGNAL;
#else
    const int send_flags = 0;
#endif

    while (size > 0) {
        ssize_t sent = send(fd, bytes, size, send_flags);

        if (sent < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -errno;
        }
        if (sent == 0) {
            return -EPIPE;
        }

        bytes += sent;
        size -= (size_t)sent;
    }

    return 0;
}

static void close_stream_fd(int *fd)
{
    if (*fd >= 0) {
        shutdown(*fd, SHUT_RDWR);
        close(*fd);
        *fd = -1;
    }
}

static void draw_red_marker(guint8 *rgb,
                            int width,
                            int height,
                            int stride,
                            int center_x,
                            int center_y)
{
    const int radius = 5;

    for (int y = center_y - radius; y <= center_y + radius; ++y) {
        for (int x = center_x - radius; x <= center_x + radius; ++x) {
            int dx = x - center_x;
            int dy = y - center_y;
            guint8 *pixel;

            if (x < 0 || x >= width || y < 0 || y >= height) {
                continue;
            }
            if (dx * dx + dy * dy > radius * radius &&
                dx != 0 && dy != 0) {
                continue;
            }

            pixel = rgb + y * stride + x * 3;
            pixel[0] = 255;
            pixel[1] = 0;
            pixel[2] = 0;
        }
    }
}

static void publish_stream_frame(VisionTracker *tracker,
                                 const guint8 *rgb,
                                 int width,
                                 int height,
                                 int stride,
                                 int marker_valid,
                                 int marker_x,
                                 int marker_y)
{
    guint row_size = (guint)(((width * 3) + 3) & ~3);
    gsize pixel_data_size = (gsize)row_size * (gsize)height;
    gsize frame_size = JIWY_VISION_BMP_HEADER_SIZE + pixel_data_size;
    guint8 *frame;

    if (!tracker->stream_enabled || !tracker->stream_running) {
        return;
    }

    frame = (guint8 *)malloc(frame_size);
    if (frame == NULL) {
        return;
    }

    memset(frame, 0, frame_size);
    frame[0] = 'B';
    frame[1] = 'M';
    write_le32(frame + 2, (uint32_t)frame_size);
    write_le32(frame + 10, JIWY_VISION_BMP_HEADER_SIZE);
    write_le32(frame + 14, 40);
    write_le32(frame + 18, (uint32_t)width);
    write_le32(frame + 22, (uint32_t)height);
    write_le16(frame + 26, 1);
    write_le16(frame + 28, 24);
    write_le32(frame + 34, (uint32_t)pixel_data_size);

    for (int y = 0; y < height; ++y) {
        const guint8 *src = rgb + y * stride;
        guint8 *dst = frame + JIWY_VISION_BMP_HEADER_SIZE +
            (gsize)(height - 1 - y) * row_size;

        for (int x = 0; x < width; ++x) {
            dst[x * 3 + 0] = src[x * 3 + 2];
            dst[x * 3 + 1] = src[x * 3 + 1];
            dst[x * 3 + 2] = src[x * 3 + 0];
        }
    }

    if (marker_valid) {
        int bmp_stride = (int)row_size;
        int bmp_y = height - 1 - marker_y;
        guint8 *bmp_rgb = (guint8 *)malloc((gsize)height * (gsize)bmp_stride);

        /*
         * Reuse the RGB marker routine on a temporary top-down view, then copy
         * it back to BMP BGR. This keeps the marker clipping logic single-use.
         */
        if (bmp_rgb != NULL) {
            memset(bmp_rgb, 0, (gsize)height * (gsize)bmp_stride);
            for (int y = 0; y < height; ++y) {
                guint8 *dst = bmp_rgb + y * bmp_stride;
                guint8 *src = frame + JIWY_VISION_BMP_HEADER_SIZE +
                    (gsize)(height - 1 - y) * row_size;
                for (int x = 0; x < width; ++x) {
                    dst[x * 3 + 0] = src[x * 3 + 2];
                    dst[x * 3 + 1] = src[x * 3 + 1];
                    dst[x * 3 + 2] = src[x * 3 + 0];
                }
            }
            (void)bmp_y;
            draw_red_marker(bmp_rgb,
                            width,
                            height,
                            bmp_stride,
                            marker_x,
                            marker_y);
            for (int y = 0; y < height; ++y) {
                guint8 *src = bmp_rgb + y * bmp_stride;
                guint8 *dst = frame + JIWY_VISION_BMP_HEADER_SIZE +
                    (gsize)(height - 1 - y) * row_size;
                for (int x = 0; x < width; ++x) {
                    dst[x * 3 + 0] = src[x * 3 + 2];
                    dst[x * 3 + 1] = src[x * 3 + 1];
                    dst[x * 3 + 2] = src[x * 3 + 0];
                }
            }
            free(bmp_rgb);
        }
    }

    pthread_mutex_lock(&tracker->stream_lock);
    free(tracker->stream_frame);
    tracker->stream_frame = frame;
    tracker->stream_frame_size = frame_size;
    tracker->stream_frame_width = width;
    tracker->stream_frame_height = height;
    ++tracker->stream_frame_sequence;
    pthread_cond_broadcast(&tracker->stream_cond);
    pthread_mutex_unlock(&tracker->stream_lock);
}

static int send_stream_http_header(int client_fd)
{
    const char *header =
        "HTTP/1.0 200 OK\r\n"
        "Server: jiwy-vision-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary="
        JIWY_VISION_STREAM_BOUNDARY "\r\n"
        "\r\n";

    return send_all(client_fd, header, strlen(header));
}

static int stream_client_loop(VisionTracker *tracker, int client_fd)
{
    uint64_t last_sequence = 0;
    int result = send_stream_http_header(client_fd);

    if (result < 0) {
        return result;
    }

    while (tracker->stream_running) {
        guint8 *frame = NULL;
        size_t frame_size = 0;
        char part_header[160];
        int header_size;

        pthread_mutex_lock(&tracker->stream_lock);
        while (tracker->stream_running &&
               tracker->stream_frame_sequence == last_sequence) {
            pthread_cond_wait(&tracker->stream_cond, &tracker->stream_lock);
        }
        if (!tracker->stream_running) {
            pthread_mutex_unlock(&tracker->stream_lock);
            break;
        }

        frame_size = tracker->stream_frame_size;
        frame = (guint8 *)malloc(frame_size);
        if (frame != NULL) {
            memcpy(frame, tracker->stream_frame, frame_size);
            last_sequence = tracker->stream_frame_sequence;
        }
        pthread_mutex_unlock(&tracker->stream_lock);

        if (frame == NULL) {
            return -ENOMEM;
        }

        header_size = snprintf(part_header,
                               sizeof(part_header),
                               "--" JIWY_VISION_STREAM_BOUNDARY "\r\n"
                               "Content-Type: image/bmp\r\n"
                               "Content-Length: %zu\r\n\r\n",
                               frame_size);
        if (header_size < 0 || header_size >= (int)sizeof(part_header)) {
            free(frame);
            return -EINVAL;
        }

        result = send_all(client_fd, part_header, (size_t)header_size);
        if (result == 0) {
            result = send_all(client_fd, frame, frame_size);
        }
        if (result == 0) {
            result = send_all(client_fd, "\r\n", 2);
        }
        free(frame);

        if (result < 0) {
            return result;
        }
    }

    return 0;
}

static void *vision_stream_thread_main(void *arg)
{
    VisionTracker *tracker = (VisionTracker *)arg;
    int listen_fd;
    int yes = 1;
    struct sockaddr_in address;

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        fprintf(stderr, "Vision stream socket failed: %s\n", strerror(errno));
        return NULL;
    }

    tracker->stream_listen_fd = listen_fd;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(0x7F000001u);
    address.sin_port = htons((uint16_t)tracker->stream_port);

    if (bind(listen_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        fprintf(stderr,
                "Vision stream bind failed on 127.0.0.1:%d: %s\n",
                tracker->stream_port,
                strerror(errno));
        close_stream_fd(&tracker->stream_listen_fd);
        return NULL;
    }

    if (listen(listen_fd, 1) < 0) {
        fprintf(stderr, "Vision stream listen failed: %s\n", strerror(errno));
        close_stream_fd(&tracker->stream_listen_fd);
        return NULL;
    }

    printf("Vision stream listening on http://127.0.0.1:%d/\n",
           tracker->stream_port);

    while (tracker->stream_running) {
        int client_fd = accept(listen_fd, NULL, NULL);

        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (tracker->stream_running) {
                fprintf(stderr,
                        "Vision stream accept failed: %s\n",
                        strerror(errno));
            }
            break;
        }

        tracker->stream_client_fd = client_fd;
        (void)stream_client_loop(tracker, client_fd);
        close_stream_fd(&tracker->stream_client_fd);
    }

    close_stream_fd(&tracker->stream_listen_fd);
    return NULL;
}

static void pixel_to_camera_error(double object_x,
                                  double object_y,
                                  int width,
                                  int height,
                                  double *yaw_error_rad,
                                  double *pitch_error_rad)
{
    double center_x = (double)width / 2.0;
    double center_y = (double)height / 2.0;

    *yaw_error_rad =
        ((object_x - center_x) / center_x) * (JIWY_VISION_FOV_RAD / 2.0);
    *pitch_error_rad =
        -((center_y - object_y) / center_y) * (JIWY_VISION_FOV_RAD / 2.0);
}

static void update_snapshot(VisionTracker *tracker,
                            const VisionTargetSnapshot *snapshot)
{
    pthread_mutex_lock(&tracker->lock);
    tracker->latest = *snapshot;
    pthread_mutex_unlock(&tracker->lock);
}

static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data)
{
    VisionPipelineData *data = (VisionPipelineData *)user_data;
    VisionTracker *tracker = data->tracker;
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstBuffer *buffer;
    GstCaps *caps;
    GstVideoInfo info;
    GstMapInfo map;
    VisionTargetSnapshot snapshot;
    int width;
    int height;
    int stride;
    size_t pixel_count;
    VisionBlobDetection detection;
    int marker_valid = 0;
    int marker_x = 0;
    int marker_y = 0;
    struct timespec frame_start;
    struct timespec frame_end;
    long process_us = 0;
    double frame_interval_ms = 0.0;
    const double nominal_frame_ms = 1000.0 / (double)JIWY_VISION_FRAME_RATE;

    if (sample == NULL) {
        return GST_FLOW_ERROR;
    }

    if (clock_gettime(CLOCK_MONOTONIC, &frame_start) < 0) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    buffer = gst_sample_get_buffer(sample);
    caps = gst_sample_get_caps(sample);

    if (buffer == NULL || caps == NULL ||
        !gst_video_info_from_caps(&info, caps)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
        fprintf(stderr,
                "Vision tracker expected RGB frames, got %s\n",
                GST_VIDEO_INFO_NAME(&info));
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    width = GST_VIDEO_INFO_WIDTH(&info);
    height = GST_VIDEO_INFO_HEIGHT(&info);
    stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);

    if (width <= 0 || height <= 0 ||
        (size_t)height > (size_t)INT_MAX / (size_t)width) {
        fprintf(stderr, "Vision tracker got invalid frame dimensions\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    pixel_count = (size_t)width * (size_t)height;

    if (stride <= 0 || !gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (map.size < (gsize)((height - 1) * stride + width * 3)) {
        fprintf(stderr,
                "Vision tracker frame buffer is smaller than expected\n");
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    if (ensure_detection_buffers(data, pixel_count) < 0) {
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    find_green_blob(data, map.data, width, height, stride, &detection);
    smooth_blob_detection(data, &detection);

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.frame_count = data->frame_count;
    if (data->have_last_frame_time) {
        frame_interval_ms =
            (double)timespec_diff_us(frame_start, data->last_frame_time) / 1000.0;
    }
    snapshot.frame_interval_ms = frame_interval_ms;

    if (detection.valid) {
        snapshot.valid = 1;
        marker_valid = 1;
        marker_x = (int)(detection.object_x + 0.5);
        marker_y = (int)(detection.object_y + 0.5);
        pixel_to_camera_error(detection.object_x,
                              detection.object_y,
                              width,
                              height,
                              &snapshot.yaw_error_rad,
                              &snapshot.pitch_error_rad);
    }

    if (clock_gettime(CLOCK_MONOTONIC, &frame_end) < 0) {
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    process_us = timespec_diff_us(frame_end, frame_start);
    snapshot.process_us = (double)process_us;
    snapshot.late_frame =
        data->have_last_frame_time &&
        frame_interval_ms > nominal_frame_ms * 1.5;
    data->last_frame_time = frame_start;
    data->have_last_frame_time = 1;

    update_snapshot(tracker, &snapshot);

    if (tracker->stream_enabled &&
        data->frame_count % JIWY_VISION_STREAM_FPS_DIVISOR == 0) {
        publish_stream_frame(tracker,
                             map.data,
                             width,
                             height,
                             stride,
                             marker_valid,
                             marker_x,
                             marker_y);
    }

    if (tracker->debug_enabled &&
        data->frame_count % JIWY_VISION_DEBUG_EVERY_FRAMES == 0) {
        if (snapshot.valid) {
            printf("vision frame=%" G_GUINT64_FORMAT
                   " yaw_err=%.4f pitch_err=%.4f blob=%" G_GUINT64_FORMAT
                   " green=%" G_GUINT64_FORMAT
                   " box=%dx%d"
                   " dt=%.2fms proc=%.0fus late=%d\n",
                   (guint64)data->frame_count,
                   snapshot.yaw_error_rad,
                   snapshot.pitch_error_rad,
                   (guint64)detection.blob_pixels,
                   (guint64)detection.total_green_pixels,
                   detection.max_x - detection.min_x + 1,
                   detection.max_y - detection.min_y + 1,
                   snapshot.frame_interval_ms,
                   snapshot.process_us,
                   snapshot.late_frame);
        } else {
            printf("vision frame=%" G_GUINT64_FORMAT
                   " no green blob green=%" G_GUINT64_FORMAT
                   " dt=%.2fms proc=%.0fus late=%d\n",
                   (guint64)data->frame_count,
                   (guint64)detection.total_green_pixels,
                   snapshot.frame_interval_ms,
                   snapshot.process_us,
                   snapshot.late_frame);
        }
    }

    ++data->frame_count;
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

static gboolean on_bus_message(GstBus *bus,
                               GstMessage *message,
                               gpointer user_data)
{
    VisionPipelineData *data = (VisionPipelineData *)user_data;

    (void)bus;

    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
        GError *error = NULL;
        gchar *debug = NULL;

        gst_message_parse_error(message, &error, &debug);
        fprintf(stderr,
                "Vision tracker GStreamer error: %s\n",
                error != NULL ? error->message : "unknown");
        fprintf(stderr,
                "Vision tracker debug info: %s\n",
                debug != NULL ? debug : "none");
        g_clear_error(&error);
        g_free(debug);
        g_main_loop_quit(data->tracker->loop);
        break;
    }
    case GST_MESSAGE_EOS:
        g_main_loop_quit(data->tracker->loop);
        break;
    default:
        break;
    }

    return TRUE;
}

static void publish_start_result(VisionTracker *tracker, int result)
{
    pthread_mutex_lock(&tracker->state_lock);
    tracker->start_result = result;
    tracker->start_done = 1;
    pthread_cond_signal(&tracker->state_cond);
    pthread_mutex_unlock(&tracker->state_lock);
}

static void *vision_thread_main(void *arg)
{
    VisionTracker *tracker = (VisionTracker *)arg;
    VisionPipelineData data;
    GstElement *pipeline;
    GstElement *source;
    GstElement *source_capsfilter;
    GstElement *jpegdec;
    GstElement *convert;
    GstElement *scale;
    GstElement *rgb_capsfilter;
    GstElement *sink;
    GstCaps *source_caps;
    GstCaps *rgb_caps;
    GstBus *bus;
    guint bus_watch_id;
    GstStateChangeReturn state_result;
    GstStateChangeReturn ready_result;

    gst_init(NULL, NULL);

    memset(&data, 0, sizeof(data));
    data.tracker = tracker;

    pipeline = gst_pipeline_new("jiwy-vision-tracker");
    source = gst_element_factory_make("v4l2src", "source");
    source_capsfilter = gst_element_factory_make("capsfilter", "source-caps");
    jpegdec = gst_element_factory_make("jpegdec", "jpeg-decoder");
    convert = gst_element_factory_make("videoconvert", "rgb-convert");
    scale = gst_element_factory_make("videoscale", "video-scale");
    rgb_capsfilter = gst_element_factory_make("capsfilter", "rgb-caps");
    sink = gst_element_factory_make("appsink", "sink");

    if (pipeline == NULL || source == NULL || source_capsfilter == NULL ||
        jpegdec == NULL || convert == NULL || scale == NULL ||
        rgb_capsfilter == NULL || sink == NULL) {
        fprintf(stderr, "Vision tracker could not create GStreamer elements\n");
        if (pipeline != NULL) {
            gst_object_unref(pipeline);
        }
        publish_start_result(tracker, -ENODEV);
        return NULL;
    }

    g_object_set(source, "device", tracker->camera_device, NULL);

    source_caps = gst_caps_new_simple("image/jpeg",
                                      "width", G_TYPE_INT, JIWY_VISION_FRAME_WIDTH,
                                      "height", G_TYPE_INT, JIWY_VISION_FRAME_HEIGHT,
                                      "framerate", GST_TYPE_FRACTION,
                                      JIWY_VISION_FRAME_RATE, 1,
                                      NULL);
    g_object_set(source_capsfilter, "caps", source_caps, NULL);
    gst_caps_unref(source_caps);

    rgb_caps = gst_caps_new_simple("video/x-raw",
                                   "format", G_TYPE_STRING, "RGB",
                                   "width", G_TYPE_INT, JIWY_VISION_FRAME_WIDTH,
                                   "height", G_TYPE_INT, JIWY_VISION_FRAME_HEIGHT,
                                   "framerate", GST_TYPE_FRACTION,
                                   JIWY_VISION_FRAME_RATE, 1,
                                   NULL);
    g_object_set(rgb_capsfilter, "caps", rgb_caps, NULL);
    gst_caps_unref(rgb_caps);

    g_object_set(sink,
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 "max-buffers", 1,
                 "drop", TRUE,
                 NULL);
    g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample), &data);

    gst_bin_add_many(GST_BIN(pipeline),
                     source,
                     source_capsfilter,
                     jpegdec,
                     convert,
                     scale,
                     rgb_capsfilter,
                     sink,
                     NULL);

    if (!gst_element_link_many(source,
                               source_capsfilter,
                               jpegdec,
                               convert,
                               scale,
                               rgb_capsfilter,
                               sink,
                               NULL)) {
        fprintf(stderr,
                "Vision tracker could not link MJPEG camera pipeline\n");
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EINVAL);
        return NULL;
    }

    tracker->loop = g_main_loop_new(NULL, FALSE);
    tracker->pipeline = pipeline;

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, on_bus_message, &data);
    gst_object_unref(bus);

    state_result = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Vision tracker failed to start camera pipeline\n");
        g_source_remove(bus_watch_id);
        g_main_loop_unref(tracker->loop);
        tracker->loop = NULL;
        tracker->pipeline = NULL;
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EIO);
        return NULL;
    }

    ready_result = gst_element_get_state(pipeline,
                                         NULL,
                                         NULL,
                                         5 * GST_SECOND);
    if (ready_result == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "Vision tracker camera pipeline did not reach PLAYING\n");
        gst_element_set_state(pipeline, GST_STATE_NULL);
        g_source_remove(bus_watch_id);
        g_main_loop_unref(tracker->loop);
        tracker->loop = NULL;
        tracker->pipeline = NULL;
        gst_object_unref(pipeline);
        publish_start_result(tracker, -EIO);
        return NULL;
    }

    publish_start_result(tracker, 0);
    g_main_loop_run(tracker->loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    g_source_remove(bus_watch_id);
    g_main_loop_unref(tracker->loop);
    tracker->loop = NULL;
    tracker->pipeline = NULL;
    gst_object_unref(pipeline);
    free(data.green_mask);
    free(data.component_queue);

    return NULL;
}

void vision_tracker_init(VisionTracker *tracker)
{
    memset(tracker, 0, sizeof(*tracker));
    pthread_mutex_init(&tracker->lock, NULL);
    pthread_mutex_init(&tracker->state_lock, NULL);
    pthread_mutex_init(&tracker->stream_lock, NULL);
    pthread_cond_init(&tracker->state_cond, NULL);
    pthread_cond_init(&tracker->stream_cond, NULL);
    tracker->stream_listen_fd = -1;
    tracker->stream_client_fd = -1;
}

int vision_tracker_start(VisionTracker *tracker,
                         const char *camera_device,
                         int debug_enabled,
                         int stream_enabled,
                         int stream_port)
{
    int result;

    if (tracker == NULL) {
        return -EINVAL;
    }

    tracker->camera_device =
        camera_device != NULL ? camera_device : JIWY_VISION_DEFAULT_CAMERA;
    tracker->debug_enabled = debug_enabled;
    tracker->stream_enabled = stream_enabled;
    tracker->stream_port =
        stream_port > 0 ? stream_port : JIWY_VISION_STREAM_PORT;
    tracker->start_done = 0;
    tracker->start_result = 0;
    tracker->stream_running = stream_enabled;
    tracker->stream_thread_started = 0;

    if (tracker->stream_enabled) {
        result = pthread_create(&tracker->stream_thread,
                                NULL,
                                vision_stream_thread_main,
                                tracker);
        if (result != 0) {
            tracker->stream_running = 0;
            return -result;
        }
        tracker->stream_thread_started = 1;
    }

    result = pthread_create(&tracker->thread,
                            NULL,
                            vision_thread_main,
                            tracker);
    if (result != 0) {
        tracker->stream_running = 0;
        pthread_cond_broadcast(&tracker->stream_cond);
        close_stream_fd(&tracker->stream_listen_fd);
        if (tracker->stream_thread_started) {
            pthread_join(tracker->stream_thread, NULL);
            tracker->stream_thread_started = 0;
        }
        return -result;
    }
    tracker->thread_started = 1;

    pthread_mutex_lock(&tracker->state_lock);
    while (!tracker->start_done) {
        pthread_cond_wait(&tracker->state_cond, &tracker->state_lock);
    }
    result = tracker->start_result;
    pthread_mutex_unlock(&tracker->state_lock);

    if (result < 0) {
        pthread_join(tracker->thread, NULL);
        tracker->thread_started = 0;
        tracker->stream_running = 0;
        pthread_cond_broadcast(&tracker->stream_cond);
        close_stream_fd(&tracker->stream_listen_fd);
        close_stream_fd(&tracker->stream_client_fd);
        if (tracker->stream_thread_started) {
            pthread_join(tracker->stream_thread, NULL);
            tracker->stream_thread_started = 0;
        }
    } else {
        tracker->running = 1;
    }

    return result;
}

void vision_tracker_stop(VisionTracker *tracker)
{
    if (tracker == NULL || !tracker->thread_started) {
        return;
    }

    if (tracker->loop != NULL) {
        g_main_loop_quit(tracker->loop);
    }

    pthread_join(tracker->thread, NULL);
    tracker->thread_started = 0;
    tracker->running = 0;

    tracker->stream_running = 0;
    pthread_cond_broadcast(&tracker->stream_cond);
    close_stream_fd(&tracker->stream_listen_fd);
    close_stream_fd(&tracker->stream_client_fd);
    if (tracker->stream_thread_started) {
        pthread_join(tracker->stream_thread, NULL);
        tracker->stream_thread_started = 0;
    }

    pthread_mutex_lock(&tracker->stream_lock);
    free(tracker->stream_frame);
    tracker->stream_frame = NULL;
    tracker->stream_frame_size = 0;
    pthread_mutex_unlock(&tracker->stream_lock);
}

int vision_tracker_read_latest(VisionTracker *tracker,
                               VisionTargetSnapshot *snapshot)
{
    if (tracker == NULL || snapshot == NULL) {
        return -EINVAL;
    }

    pthread_mutex_lock(&tracker->lock);
    *snapshot = tracker->latest;
    pthread_mutex_unlock(&tracker->lock);

    return 0;
}
