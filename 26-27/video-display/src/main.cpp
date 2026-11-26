#include "main.h"
#include "liblvgl/lvgl.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

// Set these to match the EXACT resolution and filename of your .rgb file on /usd/

//frame size is 480x272 for charlie kirk and 426x240 for cheesy michael
constexpr uint32_t kFrameWidth    = 480;
constexpr uint32_t kFrameHeight   = 272;
constexpr uint32_t kBytesPerPixel = 2; // RGB565
constexpr uint32_t kFrameBytes    = kFrameWidth * kFrameHeight * kBytesPerPixel;

// Tune kTargetFps using the table above based on your logged frame read time
constexpr uint32_t kTargetFps     = 12;
constexpr uint32_t kFrameDelayMs  = 1000 / kTargetFps;
constexpr uint32_t kPollTimerMs   = 8;
constexpr size_t   kStreamBufSize = 32768; // 32 KB stdio stream buffer
// current options are /usd/charlieKirk.rgb and /usd/cheesyMicheal.rgb
constexpr const char* kVideoPath  = "/usd/charlieKirk.rgb";

lv_obj_t* g_video_img  = nullptr;
lv_obj_t* g_status_lbl = nullptr;

uint8_t* g_buf_a = nullptr;
uint8_t* g_buf_b = nullptr;
const uint8_t* volatile g_front_buf = nullptr;
lv_image_dsc_t g_img_dsc;

volatile bool g_frame_dirty = false;
volatile bool g_hide_status = false;

void show_status(const char* text) {
    if (g_status_lbl) {
        lv_label_set_text(g_status_lbl, text);
    }
}

void refresh_video_cb(lv_timer_t* timer) {
    (void)timer;
    if (g_hide_status && g_status_lbl) {
        lv_obj_add_flag(g_status_lbl, LV_OBJ_FLAG_HIDDEN);
        g_hide_status = false;
    }

    const uint8_t* latest = g_front_buf;
    if (!g_frame_dirty || !g_video_img || !latest || g_img_dsc.data == latest) {
        return;
    }

    g_frame_dirty = false;
    g_img_dsc.data = latest;
    lv_image_cache_drop(&g_img_dsc);
    lv_obj_invalidate(g_video_img);
}

void video_player_task(void* param) {
    (void)param;
    std::printf("[DBG] video_player_task started, opening %s\n", kVideoPath);
    FILE* file = std::fopen(kVideoPath, "rb");
    if (!file) {
        std::printf("[ERR] fopen failed on %s: %s (errno=%d)\n", kVideoPath, std::strerror(errno), errno);
        show_status("SD open failed (/usd/)");
        pros::Task::current().remove();
        return;
    }

    uint8_t* stream_buf = static_cast<uint8_t*>(std::malloc(kStreamBufSize));
    if (stream_buf) {
        std::setvbuf(file, reinterpret_cast<char*>(stream_buf), _IOFBF, kStreamBufSize);
    }

    std::printf("[DBG] file opened OK\n");

    uint8_t* write_buf = g_buf_b;
    bool logged_first_frame = false;

    while (true) {
        uint32_t start_ms = pros::millis();
        size_t bytes_read = std::fread(write_buf, 1, kFrameBytes, file);

        if (bytes_read < kFrameBytes) {
            std::printf("[DBG] short read (%u/%u bytes), rewinding\n",
                        (unsigned)bytes_read, (unsigned)kFrameBytes);
            std::rewind(file);
            bytes_read = std::fread(write_buf, 1, kFrameBytes, file);
            if (bytes_read < kFrameBytes) {
                std::printf("[ERR] still short after rewind (%u/%u bytes)\n",
                            (unsigned)bytes_read, (unsigned)kFrameBytes);
                show_status("Short read / bad frame size");
                pros::delay(250);
                continue;
            }
        }

        uint32_t elapsed = pros::millis() - start_ms;

        // Publish completed frame and swap the back buffer for the next read
        g_front_buf = write_buf;
        write_buf = (write_buf == g_buf_a) ? g_buf_b : g_buf_a;
        g_frame_dirty = true;

        if (!logged_first_frame) {
            std::printf("[DBG] first frame read OK (%u bytes in %u ms)\n",
                        (unsigned)bytes_read, (unsigned)elapsed);
            logged_first_frame = true;
            g_hide_status = true;
        }

        if (elapsed < kFrameDelayMs) {
            pros::delay(kFrameDelayMs - elapsed);
        } else {
            pros::delay(2);
        }
    }
}

} // namespace

void initialize() {
    std::printf("[DBG] initialize() start\n");
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

    g_status_lbl = lv_label_create(scr);
    lv_obj_set_style_text_color(g_status_lbl, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(g_status_lbl, LV_ALIGN_TOP_LEFT, 8, 8);
    lv_label_set_text(g_status_lbl, "Loading...");

    g_buf_a = static_cast<uint8_t*>(std::calloc(1, kFrameBytes));
    g_buf_b = static_cast<uint8_t*>(std::calloc(1, kFrameBytes));
    std::printf("[DBG] g_buf_a=%p, g_buf_b=%p (each %u bytes)\n",
                (void*)g_buf_a, (void*)g_buf_b, (unsigned)kFrameBytes);
    if (!g_buf_a || !g_buf_b) {
        std::printf("[ERR] double buffer allocation failed\n");
        std::free(g_buf_a);
        std::free(g_buf_b);
        g_buf_a = nullptr;
        g_buf_b = nullptr;
        show_status("Frame malloc failed");
        return;
    }

    g_front_buf = g_buf_a;

    std::memset(&g_img_dsc, 0, sizeof(g_img_dsc));
    g_img_dsc.header.w = kFrameWidth;
    g_img_dsc.header.h = kFrameHeight;
    g_img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    g_img_dsc.header.stride = kFrameWidth * kBytesPerPixel;
    g_img_dsc.data_size = kFrameBytes;
    g_img_dsc.data = g_front_buf;

    g_video_img = lv_image_create(scr);
    lv_image_set_src(g_video_img, &g_img_dsc);
    lv_obj_align(g_video_img, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_timer_create(refresh_video_cb, kPollTimerMs, nullptr);

    std::printf("[DBG] spawning video_player_task\n");
    pros::Task player(video_player_task);
}