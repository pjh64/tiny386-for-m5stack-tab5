/*
 * ST7123 Touch-Input für M5Stack Tab5 (tiny386)
 * - Relativer Maus-Modus mit Tap-to-Click
 * - Kurzes Tippen ohne Bewegung = Klick
 * - Längeres Halten oder Ziehen = kein Klick (nur Drag)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "touch_test.h"
#include "common.h"
#include "../../i8042.h"

#define TAG "TOUCH"

extern i2c_master_bus_handle_t tab5_get_i2c_bus(void);

#define ST7123_I2C_ADDR           0x55
#define ST7123_MAX_TOUCHES_REG    0x0009
#define ST7123_ADV_INFO_REG       0x0010
#define ST7123_REPORT_COORD_0_REG 0x0014
#define MAX_TOUCH_POINTS          10

/* Kalibrierung */
#define CAL_RAWX_TOP     16
#define CAL_RAWX_BOTTOM  703
#define CAL_RAWY_LEFT    1279
#define CAL_RAWY_RIGHT   13

#define VGA_W 640
#define VGA_H 480

/* Tap-to-Click Parameter */
#define TAP_MAX_TIME_MS     180    /* Max. Dauer für Tap */
#define TAP_MAX_DISTANCE    8      /* Max. Bewegung in Pixeln für Tap */

typedef struct {
    uint8_t x_h: 6;
    uint8_t reserved: 1;
    uint8_t valid: 1;
    uint8_t x_l;
    uint8_t y_h;
    uint8_t y_l;
    uint8_t area;
    uint8_t intensity;
    uint8_t reserved2;
} __attribute__((packed)) touch_report_t;

typedef struct {
    uint8_t reserved_0_1: 2;
    uint8_t with_prox: 1;
    uint8_t with_coord: 1;
    uint8_t prox_status: 3;
    uint8_t rst_chip: 1;
} __attribute__((packed)) adv_info_t;

static i2c_master_dev_handle_t s_dev = NULL;
static int s_last_vx = -1, s_last_vy = -1;
static bool s_down = false;
static int64_t s_down_time = 0;
static int s_down_vx = 0, s_down_vy = 0;

static bool read_reg(uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return i2c_master_transmit_receive(s_dev, reg_buf, 2, data, len, 100) == ESP_OK;
}

static int get_touch_raw(int *out_x, int *out_y)
{
    adv_info_t adv_info;
    if (!read_reg(ST7123_ADV_INFO_REG, (uint8_t *)&adv_info, 1)) return 0;
    if (!adv_info.with_coord) return 0;

    uint8_t max_touches = 0;
    if (!read_reg(ST7123_MAX_TOUCHES_REG, &max_touches, 1)) return 0;
    if (max_touches > MAX_TOUCH_POINTS) max_touches = MAX_TOUCH_POINTS;
    if (max_touches == 0) return 0;

    touch_report_t reports[MAX_TOUCH_POINTS];
    if (!read_reg(ST7123_REPORT_COORD_0_REG, (uint8_t *)reports,
                  sizeof(touch_report_t) * max_touches)) return 0;

    for (int i = 0; i < max_touches; i++) {
        if (!reports[i].valid) continue;
        *out_x = (reports[i].x_h << 8) | reports[i].x_l;
        *out_y = (reports[i].y_h << 8) | reports[i].y_l;
        return 1;
    }
    return 0;
}

static void raw_to_vga(int raw_x, int raw_y, int *vx, int *vy)
{
    int x = (CAL_RAWY_LEFT - raw_y) * (VGA_W - 1) / (CAL_RAWY_LEFT - CAL_RAWY_RIGHT);
    int y = (raw_x - CAL_RAWX_TOP) * (VGA_H - 1) / (CAL_RAWX_BOTTOM - CAL_RAWX_TOP);

    if (x < 0) x = 0;
    if (x > VGA_W - 1) x = VGA_W - 1;
    if (y < 0) y = 0;
    if (y > VGA_H - 1) y = VGA_H - 1;

    *vx = x;
    *vy = y;
}

static int distance(int x1, int y1, int x2, int y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    return (dx * dx) + (dy * dy);
}

static void touch_task(void *arg)
{
    ESP_LOGI(TAG, "ST7123 touch task started (tap-to-click mode)");

    while (1) {
        int raw_x = 0, raw_y = 0;
        int found = get_touch_raw(&raw_x, &raw_y);

        if (found) {
            int vx, vy;
            raw_to_vga(raw_x, raw_y, &vx, &vy);

            if (!s_down) {
                /* DOWN: Berührung beginnt, Timer und Startposition merken */
                s_down = true;
                s_down_time = esp_timer_get_time() / 1000;
                s_down_vx = vx;
                s_down_vy = vy;
                s_last_vx = vx;
                s_last_vy = vy;
                /* Noch kein Klick - erst beim Loslassen entscheiden */
            } else {
                /* MOVE: Finger zieht -> Cursor folgt relativ (kein Button-Druck) */
                int dx = vx - s_last_vx;
                int dy = vy - s_last_vy;
                if (dx != 0 || dy != 0) {
                    if (globals.mouse) {
                        ps2_mouse_event(globals.mouse, dx, dy, 0, 0x00);
                    }
                    s_last_vx = vx;
                    s_last_vy = vy;
                }
            }
        } else if (s_down) {
            /* UP: Finger weg -> entscheiden ob Tap oder nur Drag */
            s_down = false;
            
            int64_t now = esp_timer_get_time() / 1000;
            int64_t duration = now - s_down_time;
            int dist_sq = distance(s_down_vx, s_down_vy, s_last_vx, s_last_vy);
            
            /* Tap = kurz UND wenig Bewegung */
            bool is_tap = (duration <= TAP_MAX_TIME_MS) && 
                          (dist_sq <= TAP_MAX_DISTANCE * TAP_MAX_DISTANCE);
            
            if (is_tap) {
                /* Klick: Down + Up schnell hintereinander */
                if (globals.mouse) {
                    ps2_mouse_event(globals.mouse, 0, 0, 0, 0x01);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    ps2_mouse_event(globals.mouse, 0, 0, 0, 0x00);
                }
            }
            /* Sonst: kein Klick, nur der Drag war aktiv */
            
            s_last_vx = -1;
            s_last_vy = -1;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void touch_test_init(void)
{
    i2c_master_bus_handle_t bus = tab5_get_i2c_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not available");
        return;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ST7123_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ST7123 device");
        return;
    }

    xTaskCreate(touch_task, "touch_task", 4096, NULL, 5, NULL);
}
