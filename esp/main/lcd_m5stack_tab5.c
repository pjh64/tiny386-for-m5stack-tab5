#ifdef USE_LCD_M5STACK_TAB5

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_st7123.h"
#include "esp_ldo_regulator.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_cpu.h"
#include "common.h"

extern void pc_vga_step(void *pc);
extern int pc_vga_idle(void *pc);

static const char *TAG = "lcd_tab5";

/* Global DPI framebuffer pointer */
static uint16_t *dpi_fb = NULL;

/* ---- I2C / IO expander (official M5Stack Tab5 wiring) ---- */
#define I2C_SDA          31
#define I2C_SCL          32
#define I2C_FREQ         400000
#define IO_EXP1_ADDR     0x43   /* LCD/TP/camera reset, speaker, ext 5V */
#define IO_EXP2_ADDR     0x44   /* WLAN power, USB 5V, charging */

/* PI4IOE5V6408 registers */
#define PI4IO_CHIP_RESET 0x01
#define PI4IO_IO_DIR     0x03
#define PI4IO_OUT_SET    0x05
#define PI4IO_OUT_H_IM   0x07
#define PI4IO_IN_DEF_STA 0x09
#define PI4IO_PULL_EN    0x0B
#define PI4IO_PULL_SEL   0x0D
#define PI4IO_INT_MASK   0x11

static i2c_master_bus_handle_t s_i2c_bus  = NULL;
static i2c_master_dev_handle_t s_io_exp   = NULL;
static i2c_master_dev_handle_t s_io_exp2  = NULL;

static esp_err_t pi4ioe1_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_io_exp, buf, 2, 50);
}

static esp_err_t pi4ioe2_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_io_exp2, buf, 2, 50);
}

static esp_err_t io_expander_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = 0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "I2C bus");

    i2c_device_config_t dev_cfg1 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IO_EXP1_ADDR,
        .scl_speed_hz = I2C_FREQ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg1, &s_io_exp), TAG, "IO exp1");

    i2c_device_config_t dev_cfg2 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IO_EXP2_ADDR,
        .scl_speed_hz = I2C_FREQ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg2, &s_io_exp2), TAG, "IO exp2");

    /* ---- official sequence for expander #1 (0x43) ---- */
    pi4ioe1_write(PI4IO_CHIP_RESET, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    pi4ioe1_write(PI4IO_IO_DIR,   0b01111111);  /* P0-P6 out, P7 in */
    pi4ioe1_write(PI4IO_OUT_H_IM, 0b00000000);  /* no high-Z */
    pi4ioe1_write(PI4IO_PULL_SEL, 0b01111111);  /* pull up */
    pi4ioe1_write(PI4IO_PULL_EN,  0b01111111);

    /* ---- official sequence for expander #2 (0x44) ---- */
    pi4ioe2_write(PI4IO_CHIP_RESET, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    pi4ioe2_write(PI4IO_IO_DIR,     0b10111001);
    pi4ioe2_write(PI4IO_OUT_H_IM,   0b00000110);
    pi4ioe2_write(PI4IO_PULL_SEL,   0b10111001);
    pi4ioe2_write(PI4IO_PULL_EN,    0b11111001);
    pi4ioe2_write(PI4IO_IN_DEF_STA, 0b01000000);
    pi4ioe2_write(PI4IO_INT_MASK,   0b10111111);
    pi4ioe2_write(PI4IO_OUT_SET,    0b00001001); /* P0 WLAN_PWR, P3 USB5V on */

    ESP_LOGI(TAG, "IO expanders configured (official register map)");
    return ESP_OK;
}

static void lcd_hw_reset(void)
{
    /* OUT_SET register 0x05: P4=LCD_RST, P5=TP_RST, P6=CAM_RST
       official final value = 0b01110110 (0x76) */
    pi4ioe1_write(PI4IO_OUT_SET, 0b01100110);  /* LCD_RST low, rest on */
    vTaskDelay(pdMS_TO_TICKS(20));
    pi4ioe1_write(PI4IO_OUT_SET, 0b01110110);  /* LCD_RST high -> release */
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "LCD reset released via PI4IOE P4");
}

/* ---- Backlight: GPIO22 LEDC ---- */
#define BL_GPIO 22
static void backlight_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BL_GPIO,
        .duty = 410,  // ~10% Helligkeit (4095 * 0.1)
        .hpoint = 0,
    };
    ledc_channel_config(&ch);
    ESP_LOGI(TAG, "Backlight ON (GPIO%d)", BL_GPIO);
}

/* ---- ST7123 vendor init (official M5Stack Tab5 sequence) ---- */
static const st7123_lcd_init_cmd_t st7123_init_cmds[] = {
    {0x60, (uint8_t[]){0x71,0x23,0xa2}, 3, 0},
    {0x60, (uint8_t[]){0x71,0x23,0xa3}, 3, 0},
    {0x60, (uint8_t[]){0x71,0x23,0xa4}, 3, 0},
    {0xA4, (uint8_t[]){0x31}, 1, 0},
    {0xD7, (uint8_t[]){0x10,0x0A,0x10,0x2A,0x80,0x80}, 6, 0},
    {0x90, (uint8_t[]){0x71,0x23,0x5A,0x20,0x24,0x09,0x09}, 7, 0},
    {0xA3, (uint8_t[]){0x80,0x01,0x88,0x30,0x05,0x00,0x00,0x00,0x00,0x00,0x46,0x00,0x00,
                       0x1E,0x5C,0x1E,0x80,0x00,0x4F,0x05,0x00,0x00,0x00,0x00,0x00,0x46,
                       0x00,0x00,0x1E,0x5C,0x1E,0x80,0x00,0x6F,0x58,0x00,0x00,0x00,0xFF}, 40, 0},
    {0xA6, (uint8_t[]){0x03,0x00,0x24,0x55,0x36,0x00,0x39,0x00,0x6E,0x6E,0x91,0xFF,0x00,0x24,
                       0x55,0x38,0x00,0x37,0x00,0x6E,0x6E,0x91,0xFF,0x00,0x24,0x11,0x00,0x00,
                       0x00,0x00,0x6E,0x6E,0x91,0xFF,0x00,0xEC,0x11,0x00,0x03,0x00,0x03,0x6E,
                       0x6E,0xFF,0xFF,0x00,0x08,0x80,0x08,0x80,0x06,0x00,0x00,0x00,0x00}, 55, 0},
    {0xA7, (uint8_t[]){0x19,0x19,0x80,0x64,0x40,0x07,0x16,0x40,0x00,0x44,0x03,0x6E,0x6E,0x91,0xFF,
                       0x08,0x80,0x64,0x40,0x25,0x34,0x40,0x00,0x02,0x01,0x6E,0x6E,0x91,0xFF,0x08,
                       0x80,0x64,0x40,0x00,0x00,0x40,0x00,0x00,0x00,0x6E,0x6E,0x91,0xFF,0x08,0x80,
                       0x64,0x40,0x00,0x00,0x00,0x00,0x20,0x00,0x6E,0x6E,0x84,0xFF,0x08,0x80,0x44}, 60, 0},
    {0xAC, (uint8_t[]){0x03,0x19,0x19,0x18,0x18,0x06,0x13,0x13,0x11,0x11,0x08,0x08,0x0A,0x0A,0x1C,
                       0x1C,0x07,0x07,0x00,0x00,0x02,0x02,0x01,0x19,0x19,0x18,0x18,0x06,0x12,0x12,
                       0x10,0x10,0x09,0x09,0x0B,0x0B,0x1C,0x1C,0x07,0x07,0x03,0x03,0x01,0x01}, 44, 0},
    {0xAD, (uint8_t[]){0xF0,0x00,0x46,0x00,0x03,0x50,0x50,0xFF,0xFF,0xF0,0x40,0x06,0x01,
                       0x07,0x42,0x42,0xFF,0xFF,0x01,0x00,0x00,0xFF,0xFF,0xFF,0xFF}, 25, 0},
    {0xAE, (uint8_t[]){0xFE,0x3F,0x3F,0xFE,0x3F,0x3F,0x00}, 7, 0},
    {0xB2, (uint8_t[]){0x15,0x19,0x05,0x23,0x49,0xAF,0x03,0x2E,0x5C,0xD2,0xFF,0x10,0x20,0xFD,0x20,0xC0,0x00}, 17, 0},
    {0xE8, (uint8_t[]){0x20,0x6F,0x04,0x97,0x97,0x3E,0x04,0xDC,0xDC,0x3E,0x06,0xFA,0x26,0x3E}, 15, 0},
    {0x75, (uint8_t[]){0x03,0x04}, 2, 0},
    {0xE7, (uint8_t[]){0x3B,0x00,0x00,0x7C,0xA1,0x8C,0x20,0x1A,0xF0,0xB1,0x50,0x00,
                       0x50,0xB1,0x50,0xB1,0x50,0xD8,0x00,0x55,0x00,0xB1,0x00,0x45,
                       0xC9,0x6A,0xFF,0x5A,0xD8,0x18,0x88,0x15,0xB1,0x01,0x01,0x77}, 36, 0},
    {0xEA, (uint8_t[]){0x13,0x00,0x04,0x00,0x00,0x00,0x00,0x2C}, 8, 0},
    {0xB0, (uint8_t[]){0x22,0x43,0x11,0x61,0x25,0x43,0x43}, 7, 0},
    {0xB7, (uint8_t[]){0x00,0x00,0x73,0x73}, 4, 0},
    {0xBF, (uint8_t[]){0xA6,0xAA}, 2, 0},
    {0xA9, (uint8_t[]){0x00,0x00,0x73,0xFF,0x00,0x00,0x03,0x00,0x00,0x03}, 10, 0},
    {0xC8, (uint8_t[]){0x00,0x00,0x10,0x1F,0x36,0x00,0x5D,0x04,0x9D,0x05,0x10,0xF2,0x06,
                       0x60,0x03,0x11,0xAD,0x00,0xEF,0x01,0x22,0x2E,0x0E,0x74,0x08,0x32,
                       0xDC,0x09,0x33,0x0F,0xF3,0x77,0x0D,0xB0,0xDC,0x03,0xFF}, 37, 0},
    {0xC9, (uint8_t[]){0x00,0x00,0x10,0x1F,0x36,0x00,0x5D,0x04,0x9D,0x05,0x10,0xF2,0x06,
                       0x60,0x03,0x11,0xAD,0x00,0xEF,0x01,0x22,0x2E,0x0E,0x74,0x08,0x32,
                       0xDC,0x09,0x33,0x0F,0xF3,0x77,0x0D,0xB0,0xDC,0x03,0xFF}, 37, 0},
    {0x36, (uint8_t[]){0x63}, 1, 0}, /* official */  /* Official MADCTL */
    {0x11, (uint8_t[]){0x00}, 1, 100},
    {0x29, (uint8_t[]){0x00}, 1, 50},
    {0x35, (uint8_t[]){0x00}, 1, 100},
};

/* ---- Tiny386 required stubs ---- */
void wifi_main(void)
{
    ESP_LOGW(TAG, "WiFi not implemented on Tab5 port");
}

void esp32_send_packet(const uint8_t *pkt, int pkt_len)
{
    (void)pkt;
    (void)pkt_len;
}


/* Direkter Display-Test: Zeichnet ein einfaches Muster */
static void lcd_test_pattern(void)
{
    if (globals.panel == NULL) {
        ESP_LOGE(TAG, "lcd_test_pattern: panel is NULL");
        return;
    }
    
    ESP_LOGW(TAG, "=== LCD TEST PATTERN START ===");
    
    // Test 1: Roter Streifen oben
    uint16_t *red_line = (uint16_t *)heap_caps_malloc(1280 * 2, MALLOC_CAP_DMA);
    if (red_line) {
        for (int i = 0; i < 1280; i++) red_line[i] = 0xF800;  // Rot
        esp_err_t ret = esp_lcd_panel_draw_bitmap(globals.panel, 0, 0, 1280, 20, red_line);
        ESP_LOGW(TAG, "Test 1 (red stripe): ret=%d (%s)", ret, esp_err_to_name(ret));
        free(red_line);
    }
    
    // Test 2: Grüner Streifen Mitte
    uint16_t *green_line = (uint16_t *)heap_caps_malloc(1280 * 2, MALLOC_CAP_DMA);
    if (green_line) {
        for (int i = 0; i < 1280; i++) green_line[i] = 0x07E0;  // Grün
        esp_err_t ret = esp_lcd_panel_draw_bitmap(globals.panel, 0, 350, 1280, 370, green_line);
        ESP_LOGW(TAG, "Test 2 (green stripe): %s", ret == ESP_OK ? "OK" : "FAILED");
        free(green_line);
    }
    
    // Test 3: Blauer Streifen unten
    uint16_t *blue_line = (uint16_t *)heap_caps_malloc(1280 * 2, MALLOC_CAP_DMA);
    if (blue_line) {
        for (int i = 0; i < 1280; i++) blue_line[i] = 0x001F;  // Blau
        esp_err_t ret = esp_lcd_panel_draw_bitmap(globals.panel, 0, 700, 1280, 720, blue_line);
        ESP_LOGW(TAG, "Test 3 (blue stripe): %s", ret == ESP_OK ? "OK" : "FAILED");
        free(blue_line);
    }
    
    // Test 4: Weißes Quadrat in der Mitte
    uint16_t *white_square = (uint16_t *)heap_caps_malloc(200 * 200 * 2, MALLOC_CAP_DMA);
    if (white_square) {
        for (int i = 0; i < 200 * 200; i++) white_square[i] = 0xFFFF;  // Weiß
        esp_err_t ret = esp_lcd_panel_draw_bitmap(globals.panel, 540, 260, 740, 460, white_square);
        ESP_LOGW(TAG, "Test 4 (white square): %s", ret == ESP_OK ? "OK" : "FAILED");
        free(white_square);
    }
    
    ESP_LOGW(TAG, "=== LCD TEST PATTERN END ===");
}

/* ---- VGA -> LCD blit (first version, simple scale) ---- */
#define PANEL_H_RES 720
#define PANEL_V_RES 1280

void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src)
{
    static int draw_count = 0;
    draw_count++;
    

    if (draw_count <= 5 || (draw_count % 1000) == 0) {
        // Prüfe den Framebuffer-Inhalt
        uint16_t *fb_data = (uint16_t *)src;
        uint16_t first_pixel = fb_data[0];
        uint16_t mid_pixel = fb_data[(x_end - x_start) * (y_end - y_start) / 2];
        uint16_t last_pixel = fb_data[(x_end - x_start) * (y_end - y_start) - 1];
        ESP_LOGI(TAG, "lcd_draw #%d: x[%d-%d] y[%d-%d] src=%p pixels=[0x%04X, 0x%04X, 0x%04X]", 
                 draw_count, x_start, x_end, y_start, y_end, src, first_pixel, mid_pixel, last_pixel);
    }
    
    if (globals.panel == NULL || src == NULL) return;
    if (x_end <= x_start || y_end <= y_start) return;

    /* clamp to panel bounds */
    if (x_start < 0) x_start = 0;
    if (y_start < 0) y_start = 0;
    if (x_end > PANEL_H_RES) x_end = PANEL_H_RES;
    if (y_end > PANEL_V_RES) y_end = PANEL_V_RES;
    if (x_end <= x_start || y_end <= y_start) return;

    esp_lcd_panel_draw_bitmap(globals.panel, x_start, y_start, x_end, y_end, src);
}

/* ---- Display init task ---- */
void vga_task(void *arg)
{
    int core_id = esp_cpu_get_core_id();
    fprintf(stderr, "vga runs on core %d\n", core_id);
    ESP_LOGI(TAG, "=== M5Stack Tab5 ST7123 Display Init ===");

    ESP_ERROR_CHECK(io_expander_init());
    lcd_hw_reset();
    backlight_init();

    /* MIPI DSI PHY power (LDO ch3, 2.5V) */
    esp_ldo_channel_handle_t ldo_phy = NULL;
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_phy));
    ESP_LOGI(TAG, "DSI PHY LDO enabled");
    vTaskDelay(pdMS_TO_TICKS(50));

    /* DSI bus (official: 2 lanes, 965 Mbps) */
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id = 0,
        .num_data_lanes = 2,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = 965,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));
    ESP_LOGI(TAG, "DSI bus created");

    /* DBI control IO */
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &io));

    /* DPI config (official Tab5 ST7123 timings, portrait 720x1280) */
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 70,
        .virtual_channel = 0,
        .num_fbs = 1,  /* MINDESTENS 1 Framebuffer für DPI-Stream */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
        .in_color_format = LCD_COLOR_FMT_RGB565,
#else
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
#endif
        .video_timing = {
            .h_size = 720,
            .v_size = 1280,
            .hsync_back_porch = 40,
            .hsync_pulse_width = 2,
            .hsync_front_porch = 40,
            .vsync_back_porch = 8,
            .vsync_pulse_width = 2,
            .vsync_front_porch = 220,
        },
    };

    st7123_vendor_config_t vendor_cfg = {
        .init_cmds = st7123_init_cmds,
        .init_cmds_size = sizeof(st7123_init_cmds) / sizeof(st7123_init_cmds[0]),
        .mipi_config = {
            .dsi_bus = dsi_bus,
            .dpi_config = &dpi_cfg,
        },
    };

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };

    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7123(io, &panel_cfg, &panel));
    ESP_LOGI(TAG, "Panel created, sending official init sequence...");

    /* Initialisiere das Panel via Treiber (sendet Init-Commands automatisch) */
    ESP_LOGI(TAG, "Calling esp_lcd_panel_init()...");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_LOGI(TAG, "=== PANEL INITIALIZED VIA DRIVER ===");

    /* Hardware-Rotation gegen die 90-Grad-Drehung (MADCTL via Treiber).
     * Falls falsch herum: mirror-Argumente gemaess Tabelle unten tauschen. */

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    // esp_lcd_dpi_panel_enable_dma2d(panel);  // deaktiviert: DMA2D-Pool-Overflow bei hohem Tempo
    
    /* Framebuffer erst NACH Panel-Init verfügbar */
    void *fb_ptr = NULL;
    esp_err_t fb_err = esp_lcd_st7123_get_frame_buffer(panel, 1, &fb_ptr);
    if (fb_err == ESP_OK && fb_ptr != NULL) {
        ESP_LOGI(TAG, "DPI framebuffer at %p (zero-copy mode)", fb_ptr);
        globals.panel_fb = fb_ptr;
    } else {
        ESP_LOGW(TAG, "Could not get DPI framebuffer (err=%s), will use copy mode", esp_err_to_name(fb_err));
        globals.panel_fb = NULL;
    }
    
    ESP_LOGI(TAG, "=== DISPLAY FULLY INITIALIZED ===");

    globals.panel = panel;
    
    xEventGroupSetBits(global_event_group, BIT1);
    xEventGroupWaitBits(global_event_group, BIT0, pdFALSE, pdFALSE, portMAX_DELAY);

    while (1) {
        int64_t va = esp_timer_get_time();
        pc_vga_step(globals.pc);
        int64_t vb = esp_timer_get_time();
        static int vc = 0; static int64_t vacc = 0;
        vacc += vb - va;
        if (++vc >= 100) {
            ESP_LOGW("PERF", "vga_step avg us=%ld", (long)(vacc / vc));
            vc = 0; vacc = 0;
        }
        if (pc_vga_idle(globals.pc)) vTaskDelay(1);
    }
}

#endif /* USE_LCD_M5STACK_TAB5 */
