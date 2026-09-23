// M5Stack Tab5 — ESP32-P4, 32MB PSRAM, ST7123 MIPI-DSI 1280x720
// Requires ESP-IDF v6.0.x

#define BUILD_ESP32

#define PSRAM_ALLOC_LEN (10 * 1024 * 1024)
#define IRAM_ATTR_CPU_EXEC1

#define BPP 16

// Display: ST7123 via MIPI-DSI (portrait 720x1280, logically 1280x720)
#define USE_LCD_M5STACK_TAB5
#define LCD_WIDTH  1280
#define LCD_HEIGHT 720

// Audio: ES8388 codec on Tab5
#define USE_ES8388
#define I2S_MCLK  30
#define I2S_BCLK  27
#define I2S_WS    29
#define I2S_DOUT  26
#define PI4IO1_I2C_ADDR  0x43

#define MIXER_BUF_LEN 1024

/* Tab5: SD slot power rail (official BSP uses LDO_VO4 = channel 4) */
#define SD_PWR_CTRL_LDO_IO_ID 4

/* Tab5 SD card in SPI mode */
/* #define SD_SPI_SCK   43 */
/* #define SD_SPI_MOSI  44 */
/* #define SD_SPI_MISO  39 */
/* #define SD_SPI_CS    42 */
/* #define SD_SPI_FREQ_KHZ 20000 */

/* SDIO mode pins - M5Stack Tab5 microSD (25x faster than SPI) */
#define SD_CLK       43
#define SD_CMD       44
#define SD_D0        39
#define SD_D1        40
#define SD_D2        41
#define SD_D3        42

/* I2C pins used by storage.c SPI-branch pre-init (harmless on Tab5) */
#define LCD_I2C_SDA  31
#define LCD_I2C_SCL  32

#define VGA_FB_W 720
#define VGA_FB_H 480
