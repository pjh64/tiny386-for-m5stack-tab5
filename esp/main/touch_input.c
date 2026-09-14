#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#include "driver/i2c_master.h"
#include "touch_input.h"
#include "common.h"
#include "../../i8042.h"

#define TAG "TOUCH"
#define TOUCH_INT_GPIO    23
#define GT911_ADDR        0x14
#define ST712X_ADDR       0x55
#define DISPLAY_WIDTH     1280
#define DISPLAY_HEIGHT    720
#define VGA_WIDTH         640
#define VGA_HEIGHT        480

i2c_master_bus_handle_t tab5_get_i2c_bus(void);
static i2c_master_bus_handle_t s_bus = NULL;

static esp_lcd_touch_handle_t touch_handle = NULL;
static i2c_master_dev_handle_t s_touch_dev = NULL;
static i2c_master_dev_handle_t touch_dev_handle = NULL;
static bool touch_initialized = false;
static int last_x = VGA_WIDTH / 2;
static int last_y = VGA_HEIGHT / 2;
static bool mouse_pressed = false;

static esp_err_t detect_touch_controller(uint8_t *addr) {
    static const uint8_t addrs[] = { 0x14, 0x5D, 0x55 };
    for (unsigned i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = 400000,
        };
        i2c_master_dev_handle_t h = NULL;
        if (i2c_master_bus_add_device(s_bus, &cfg, &h) != ESP_OK) continue;
        uint8_t d = 0;
        esp_err_t r = i2c_master_transmit(h, &d, 1, 100);
        if (r == ESP_OK) {
            *addr = addrs[i];
            s_touch_dev = h;  // Device-Handle speichern
            ESP_LOGI(TAG, "Touch controller detected at 0x%02x", addrs[i]);
            return ESP_OK;
        }
        i2c_master_bus_rm_device(h);
    }
    return ESP_FAIL;
}

static void touch_task(void *arg) {
    ESP_LOGI(TAG, "Touch task started");
    
    while (1) {
        if (!touch_handle) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        esp_err_t read_ret = esp_lcd_touch_read_data(touch_handle);
        ESP_LOGD(TAG, "Read data result: %d", read_ret);
        
        uint16_t x[5], y[5];
        uint8_t point_num = 0;
        esp_err_t ret = esp_lcd_touch_get_coordinates(touch_handle, x, y, NULL, &point_num, 5);
        
        ESP_LOGD(TAG, "Get coordinates result: %d, point_num: %d", ret, point_num);
        
        if (ret == ESP_OK && point_num > 0) {
            ESP_LOGI(TAG, "Touch detected: x[0]=%d, y[0]=%d", x[0], y[0]);
            
            int vga_x = (x[0] * VGA_WIDTH) / DISPLAY_WIDTH;
            int vga_y = (y[0] * VGA_HEIGHT) / DISPLAY_HEIGHT;
            
            if (vga_x < 0) vga_x = 0;
            if (vga_x >= VGA_WIDTH) vga_x = VGA_WIDTH - 1;
            if (vga_y < 0) vga_y = 0;
            if (vga_y >= VGA_HEIGHT) vga_y = VGA_HEIGHT - 1;
            
            int dx = vga_x - last_x;
            int dy = vga_y - last_y;
            
            if (dx != 0 || dy != 0) {
                ESP_LOGI(TAG, "Touch at (%d,%d) -> VGA (%d,%d), delta (%d,%d)", 
                         x[0], y[0], vga_x, vga_y, dx, dy);
                
                if (globals.mouse != NULL) {
                    ESP_LOGI(TAG, "Sending PS/2 mouse event: dx=%d, dy=%d", dx, dy);
                    ps2_mouse_event(globals.mouse, dx, dy, 0, 0x01);
                } else {
                    ESP_LOGW(TAG, "globals.mouse is NULL!");
                }
                
                last_x = vga_x;
                last_y = vga_y;
                mouse_pressed = true;
            }
        } else if (mouse_pressed) {
            ESP_LOGI(TAG, "Touch released");
            if (globals.mouse != NULL) {
                ps2_mouse_event(globals.mouse, 0, 0, 0, 0x00);
            }
            mouse_pressed = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t touch_input_init(void) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Initializing touch input...");
    
    s_bus = tab5_get_i2c_bus();
    if (s_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    vTaskDelay(pdMS_TO_TICKS(300));  /* Touch-IC Bootzeit nach Reset-Release */
    uint8_t addr;
    esp_err_t ret = detect_touch_controller(&addr);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No touch controller detected");
        return ESP_OK;
    }
    
    // Erstelle I2C IO Handle für Touch (verwende bereits existierendes Device-Handle)
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = addr,
        .scl_speed_hz = 400000,  // Pflichtfeld in ESP-IDF v6.0
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 16,
        .lcd_param_bits = 0,
        .flags = { .dc_low_on_data = 0, .disable_control_phase = 1 },
    };
    
    ret = esp_lcd_new_panel_io_i2c(s_bus, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C IO: %s", esp_err_to_name(ret));
        return ESP_OK;
    }
    
    // Touch-Konfiguration
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = DISPLAY_WIDTH,
        .y_max = DISPLAY_HEIGHT,
        .rst_gpio_num = -1,
        .int_gpio_num = TOUCH_INT_GPIO,
        .levels = { .reset = 0, .interrupt = 0 },
    };
    
    // Initialisiere GT911 (funktioniert auch für ST712x)
    ret = esp_lcd_touch_new_i2c_gt911(io_handle, &tp_cfg, &touch_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Touch init failed: %s", esp_err_to_name(ret));
        return ESP_OK;
    }
    
    touch_initialized = true;
    ESP_LOGI(TAG, "Touch controller initialized");
    
    xTaskCreate(touch_task, "touch_task", 4096, NULL, 5, NULL);
    
    return ESP_OK;
}

bool touch_input_is_initialized(void) {
    return touch_initialized;
}
