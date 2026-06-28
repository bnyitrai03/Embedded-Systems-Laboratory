#include "vision_stream.h"

#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "jiwy_config.h"
#include "vision_tracker.h"

#define STREAM_MAX(a, b) ((a) > (b) ? (a) : (b))

static void write_le16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void write_le32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8) & 0xFFu);
    out[2] = (uint8_t)((value >> 16) & 0xFFu);
    out[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static int send_all(int fd, const void *buffer, size_t size)
{
    const uint8_t *bytes = (const uint8_t *)buffer;
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

static void set_rgb(uint8_t *rgb, int width, int height, int stride, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t *pixel;

    if (x < 0 || x >= width || y < 0 || y >= height) {
        return;
    }

    pixel = rgb + y * stride + x * 3;
    pixel[0] = r;
    pixel[1] = g;
    pixel[2] = b;
}

static void draw_cross(uint8_t *rgb, int width, int height, int stride, int center_x, int center_y, int radius, uint8_t r, uint8_t g, uint8_t b)
{
    for (int d = -radius; d <= radius; ++d) {
        set_rgb(rgb, width, height, stride, center_x + d, center_y, r, g, b);
        set_rgb(rgb, width, height, stride, center_x, center_y + d, r, g, b);
    }
}

static void draw_circle(uint8_t *rgb, int width, int height, int stride, int center_x, int center_y, int radius, uint8_t r, uint8_t g, uint8_t b)
{
    int radius_sq = radius * radius;
    int thickness = STREAM_MAX(6, radius / 8);

    if (radius < 2) {
        return;
    }

    for (int y = center_y - radius - 1; y <= center_y + radius + 1; ++y) {
        for (int x = center_x - radius - 1; x <= center_x + radius + 1; ++x) {
            int dx = x - center_x;
            int dy = y - center_y;
            int distance_sq = dx * dx + dy * dy;

            if (abs(distance_sq - radius_sq) <= thickness) {
                set_rgb(rgb, width, height, stride, x, y, r, g, b);
            }
        }
    }
}

static uint8_t glyph_row(char ch, int row)
{
    static const uint8_t glyph_space[7] = {0, 0, 0, 0, 0, 0, 0};
    static const uint8_t glyph_dash[7] = {0, 0, 0, 31, 0, 0, 0};
    static const uint8_t glyph_dot[7] = {0, 0, 0, 0, 0, 12, 12};
    static const uint8_t glyph_0[7] = {14, 17, 19, 21, 25, 17, 14};
    static const uint8_t glyph_1[7] = {4, 12, 4, 4, 4, 4, 14};
    static const uint8_t glyph_2[7] = {14, 17, 1, 2, 4, 8, 31};
    static const uint8_t glyph_3[7] = {30, 1, 1, 14, 1, 1, 30};
    static const uint8_t glyph_4[7] = {2, 6, 10, 18, 31, 2, 2};
    static const uint8_t glyph_5[7] = {31, 16, 16, 30, 1, 1, 30};
    static const uint8_t glyph_6[7] = {14, 16, 16, 30, 17, 17, 14};
    static const uint8_t glyph_7[7] = {31, 1, 2, 4, 8, 8, 8};
    static const uint8_t glyph_8[7] = {14, 17, 17, 14, 17, 17, 14};
    static const uint8_t glyph_9[7] = {14, 17, 17, 15, 1, 1, 14};
    static const uint8_t glyph_a[7] = {14, 17, 17, 31, 17, 17, 17};
    static const uint8_t glyph_b[7] = {30, 17, 17, 30, 17, 17, 30};
    static const uint8_t glyph_c[7] = {14, 17, 16, 16, 16, 17, 14};
    static const uint8_t glyph_d[7] = {30, 17, 17, 17, 17, 17, 30};
    static const uint8_t glyph_e[7] = {31, 16, 16, 30, 16, 16, 31};
    static const uint8_t glyph_g[7] = {14, 17, 16, 23, 17, 17, 15};
    static const uint8_t glyph_h[7] = {17, 17, 17, 31, 17, 17, 17};
    static const uint8_t glyph_i[7] = {14, 4, 4, 4, 4, 4, 14};
    static const uint8_t glyph_l[7] = {16, 16, 16, 16, 16, 16, 31};
    static const uint8_t glyph_m[7] = {17, 27, 21, 21, 17, 17, 17};
    static const uint8_t glyph_n[7] = {17, 25, 21, 19, 17, 17, 17};
    static const uint8_t glyph_o[7] = {14, 17, 17, 17, 17, 17, 14};
    static const uint8_t glyph_p[7] = {30, 17, 17, 30, 16, 16, 16};
    static const uint8_t glyph_r[7] = {30, 17, 17, 30, 20, 18, 17};
    static const uint8_t glyph_s[7] = {15, 16, 16, 14, 1, 1, 30};
    static const uint8_t glyph_t[7] = {31, 4, 4, 4, 4, 4, 4};
    static const uint8_t glyph_u[7] = {17, 17, 17, 17, 17, 17, 14};
    static const uint8_t glyph_w[7] = {17, 17, 17, 21, 21, 21, 10};
    static const uint8_t glyph_y[7] = {17, 17, 10, 4, 4, 4, 4};
    const uint8_t *glyph = glyph_space;

    switch (ch) {
    case '-': glyph = glyph_dash; break;
    case '.': glyph = glyph_dot; break;
    case '0': glyph = glyph_0; break;
    case '1': glyph = glyph_1; break;
    case '2': glyph = glyph_2; break;
    case '3': glyph = glyph_3; break;
    case '4': glyph = glyph_4; break;
    case '5': glyph = glyph_5; break;
    case '6': glyph = glyph_6; break;
    case '7': glyph = glyph_7; break;
    case '8': glyph = glyph_8; break;
    case '9': glyph = glyph_9; break;
    case 'A': glyph = glyph_a; break;
    case 'B': glyph = glyph_b; break;
    case 'C': glyph = glyph_c; break;
    case 'D': glyph = glyph_d; break;
    case 'E': glyph = glyph_e; break;
    case 'G': glyph = glyph_g; break;
    case 'H': glyph = glyph_h; break;
    case 'I': glyph = glyph_i; break;
    case 'L': glyph = glyph_l; break;
    case 'M': glyph = glyph_m; break;
    case 'N': glyph = glyph_n; break;
    case 'O': glyph = glyph_o; break;
    case 'P': glyph = glyph_p; break;
    case 'R': glyph = glyph_r; break;
    case 'S': glyph = glyph_s; break;
    case 'T': glyph = glyph_t; break;
    case 'U': glyph = glyph_u; break;
    case 'W': glyph = glyph_w; break;
    case 'Y': glyph = glyph_y; break;
    default: break;
    }

    return glyph[row];
}

static void draw_text(uint8_t *rgb, int width, int height, int stride, int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b)
{
    const int scale = 2;
    int cursor_x = x;

    for (const char *p = text; *p != '\0'; ++p) {
        for (int row = 0; row < 7; ++row) {
            uint8_t bits = glyph_row(*p, row);
            for (int col = 0; col < 5; ++col) {
                if ((bits & (uint8_t)(1u << (4 - col))) == 0) {
                    continue;
                }
                for (int sy = 0; sy < scale; ++sy) {
                    for (int sx = 0; sx < scale; ++sx) {
                        set_rgb(rgb, width, height, stride, cursor_x + col * scale + sx, y + row * scale + sy, r, g, b);
                    }
                }
            }
        }
        cursor_x += 6 * scale;
    }
}

static void annotate_stream_frame(uint8_t *annotated, const uint8_t *rgb, int width, int height, int stride, const VisionBlobDetection *detection, const VisionTargetSnapshot *snapshot)
{
    char line[96];

    for (int y = 0; y < height; ++y) {
        memcpy(annotated + y * width * 3,
               rgb + y * stride,
               (size_t)width * 3u);
    }

    draw_cross(annotated,
               width,
               height,
               width * 3,
               width / 2,
               height / 2,
               9,
               60,
               160,
               255);

    if (snapshot->valid && detection->valid) {
        int center_x = (detection->min_x + detection->max_x) / 2;
        int center_y = (detection->min_y + detection->max_y) / 2;
        int blob_width = detection->max_x - detection->min_x + 1;
        int blob_height = detection->max_y - detection->min_y + 1;
        int radius = STREAM_MAX(blob_width, blob_height) / 2 + 4;

        draw_circle(annotated,
                    width,
                    height,
                    width * 3,
                    center_x,
                    center_y,
                    radius,
                    255,
                    220,
                    40);
        draw_cross(annotated,
                   width,
                   height,
                   width * 3,
                   (int)(detection->object_x + 0.5),
                   (int)(detection->object_y + 0.5),
                   8,
                   255,
                   255,
                   255);
    }

    draw_text(annotated,
              width,
              height,
              width * 3,
              8,
              8,
              snapshot->valid ? "DETECTED" : "NO TARGET",
              snapshot->valid ? 40 : 255,
              snapshot->valid ? 255 : 60,
              snapshot->valid ? 80 : 60);

    snprintf(line,
             sizeof(line),
             "YAW %.3f PITCH %.3f",
             snapshot->yaw_error_rad,
             snapshot->pitch_error_rad);
    draw_text(annotated, width, height, width * 3, 8, 28, line, 255, 255, 255);

    snprintf(line,
             sizeof(line),
             "BLOB %llu GREEN %llu",
             (unsigned long long)detection->blob_pixels,
             (unsigned long long)detection->total_green_pixels);
    draw_text(annotated, width, height, width * 3, 8, 48, line, 255, 255, 255);

    snprintf(line,
             sizeof(line),
             "PROC %.0fUS DT %.1fMS",
             snapshot->process_us,
             snapshot->frame_interval_ms);
    draw_text(annotated, width, height, width * 3, 8, 68, line, 255, 255, 255);
}

void vision_stream_publish_rgb(VisionStream *stream, const uint8_t *rgb, int width, int height, int stride, const VisionBlobDetection *detection, const VisionTargetSnapshot *snapshot)
{
    uint32_t row_size = (uint32_t)(((width * 3) + 3) & ~3);
    size_t pixel_data_size = (size_t)row_size * (size_t)height;
    size_t frame_size = JIWY_VISION_BMP_HEADER_SIZE + pixel_data_size;
    uint8_t *annotated;
    uint8_t *frame;

    if (stream == NULL || !stream->enabled || !stream->running ||
        rgb == NULL || detection == NULL || snapshot == NULL) {
        return;
    }

    annotated = (uint8_t *)malloc((size_t)width * (size_t)height * 3u);
    if (annotated == NULL) {
        return;
    }

    frame = (uint8_t *)malloc(frame_size);
    if (frame == NULL) {
        free(annotated);
        return;
    }

    annotate_stream_frame(annotated, rgb, width, height, stride, detection, snapshot);

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
        const uint8_t *src = annotated + y * width * 3;
        uint8_t *dst = frame + JIWY_VISION_BMP_HEADER_SIZE +
            (size_t)(height - 1 - y) * row_size;

        for (int x = 0; x < width; ++x) {
            dst[x * 3 + 0] = src[x * 3 + 2];
            dst[x * 3 + 1] = src[x * 3 + 1];
            dst[x * 3 + 2] = src[x * 3 + 0];
        }
    }

    pthread_mutex_lock(&stream->lock);
    free(stream->frame);
    stream->frame = frame;
    stream->frame_size = frame_size;
    ++stream->frame_sequence;
    pthread_cond_broadcast(&stream->cond);
    pthread_mutex_unlock(&stream->lock);
    free(annotated);
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

static int stream_client_loop(VisionStream *stream, int client_fd)
{
    uint64_t last_sequence = 0;
    int result = send_stream_http_header(client_fd);

    if (result < 0) {
        return result;
    }

    while (stream->running) {
        uint8_t *frame = NULL;
        size_t frame_size = 0;
        char part_header[160];
        int header_size;

        pthread_mutex_lock(&stream->lock);
        while (stream->running &&
               stream->frame_sequence == last_sequence) {
            pthread_cond_wait(&stream->cond, &stream->lock);
        }
        if (!stream->running) {
            pthread_mutex_unlock(&stream->lock);
            break;
        }

        frame_size = stream->frame_size;
        frame = (uint8_t *)malloc(frame_size);
        if (frame != NULL) {
            memcpy(frame, stream->frame, frame_size);
            last_sequence = stream->frame_sequence;
        }
        pthread_mutex_unlock(&stream->lock);

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
    VisionStream *stream = (VisionStream *)arg;
    int listen_fd;
    int yes = 1;
    struct sockaddr_in address;

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        fprintf(stderr, "Vision stream socket failed: %s\n", strerror(errno));
        return NULL;
    }

    stream->listen_fd = listen_fd;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(0x7F000001u);
    address.sin_port = htons((uint16_t)stream->port);

    if (bind(listen_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        fprintf(stderr,
                "Vision stream bind failed on 127.0.0.1:%d: %s\n",
                stream->port,
                strerror(errno));
        close_stream_fd(&stream->listen_fd);
        return NULL;
    }

    if (listen(listen_fd, 1) < 0) {
        fprintf(stderr, "Vision stream listen failed: %s\n", strerror(errno));
        close_stream_fd(&stream->listen_fd);
        return NULL;
    }

    printf("Vision stream listening on http://127.0.0.1:%d/\n",
           stream->port);

    while (stream->running) {
        int client_fd = accept(listen_fd, NULL, NULL);

        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (stream->running) {
                fprintf(stderr,
                        "Vision stream accept failed: %s\n",
                        strerror(errno));
            }
            break;
        }

        stream->client_fd = client_fd;
        (void)stream_client_loop(stream, client_fd);
        close_stream_fd(&stream->client_fd);
    }

    close_stream_fd(&stream->listen_fd);
    return NULL;
}

void vision_stream_init(VisionStream *stream)
{
    memset(stream, 0, sizeof(*stream));
    pthread_mutex_init(&stream->lock, NULL);
    pthread_cond_init(&stream->cond, NULL);
    stream->listen_fd = -1;
    stream->client_fd = -1;
}

int vision_stream_start(VisionStream *stream, int enabled, int port)
{
    int result;

    if (stream == NULL) {
        return -EINVAL;
    }

    stream->enabled = enabled;
    stream->port = port > 0 ? port : JIWY_VISION_STREAM_PORT;
    stream->running = enabled;
    stream->thread_started = 0;

    if (!stream->enabled) {
        return 0;
    }

    result = pthread_create(&stream->thread, NULL, vision_stream_thread_main, stream);
    if (result != 0) {
        stream->running = 0;
        return -result;
    }
    stream->thread_started = 1;
    return 0;
}

void vision_stream_stop(VisionStream *stream)
{
    if (stream == NULL) {
        return;
    }

    stream->running = 0;
    pthread_cond_broadcast(&stream->cond);
    close_stream_fd(&stream->listen_fd);
    close_stream_fd(&stream->client_fd);
    if (stream->thread_started) {
        pthread_join(stream->thread, NULL);
        stream->thread_started = 0;
    }

    pthread_mutex_lock(&stream->lock);
    free(stream->frame);
    stream->frame = NULL;
    stream->frame_size = 0;
    pthread_mutex_unlock(&stream->lock);
}