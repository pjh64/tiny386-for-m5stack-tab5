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

// SD card (Tab5 pinout)
// No hosted WiFi for now (ESP32-C6 co-processor needs extra setup)

// Audio: DISABLED — Tab5 uses ES8388+ES7210 (not ES8311)
// Removing USE_ES8311 prevents the bootloop assert in i2s.c
// #define USE_ES8311

#define MIXER_BUF_LEN 512

// WiFi disabled - Tab5 uses ESP32-C6 co-processor (not yet supported)

/* Tab5: SD slot power rail (official BSP uses LDO_VO4 = channel 4) */
#define SD_PWR_CTRL_LDO_IO_ID 4

/* Tab5 SD card in SPI mode (proven working by M5Tab-Macintosh).
 * SCK=43 MOSI=44 MISO=39 CS=42 */
#define SD_SPI_SCK   43
#define SD_SPI_MOSI  44
#define SD_SPI_MISO  39
#define SD_SPI_CS    42
#define SD_SPI_FREQ_KHZ 20000

/* I2C pins used by storage.c SPI-branch pre-init (harmless on Tab5) */
#define LCD_I2C_SDA  31
#define LCD_I2C_SCL  32



