#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "es8388.h"
#include "board_m5stack_tab5.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ES8388_ADDR 0x10
#define TAG "ES8388"

static i2c_master_dev_handle_t es8388_dev = NULL;
static i2c_master_dev_handle_t pi4io1_dev = NULL;

static esp_err_t es8388_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(es8388_dev, write_buf, 2, 100);
}

static esp_err_t es8388_read_reg(uint8_t reg_addr, uint8_t *data) {
    return i2c_master_transmit_receive(es8388_dev, &reg_addr, 1, data, 1, 100);
}

static esp_err_t pi4io1_write_reg(uint8_t reg_addr, uint8_t data) {
    if (pi4io1_dev == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(pi4io1_dev, write_buf, 2, 100);
}

static esp_err_t pi4io1_read_reg(uint8_t reg_addr, uint8_t *data) {
    if (pi4io1_dev == NULL) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(pi4io1_dev, &reg_addr, 1, data, 1, 100);
}

void speaker_enable(int enable) {
    /* PI4IOE5V6408 Register 0x05 = OUT_SET, Bit 1 (P1) = SPK_EN */
    uint8_t out = 0;
    esp_err_t ret = pi4io1_read_reg(0x05, &out);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read OUT_SET failed: %s", esp_err_to_name(ret));
        return;
    }
    
    if (enable) out |= 0x02;   /* Bit 1 = HIGH */
    else out &= ~0x02;         /* Bit 1 = LOW */
    
    ret = pi4io1_write_reg(0x05, out);
    ESP_LOGI(TAG, "speaker_enable(%d): OUT_SET(0x05)=0x%02X, ret=%d", 
             enable, out, ret);
}

void es8388_mute(int mute) {
    if (mute) {
        es8388_write_reg(0x19, 0x24);  /* DACCONTROL3: Soft mute */
    } else {
        es8388_write_reg(0x19, 0x20);  /* DACCONTROL3: Unmute */
    }
}

esp_err_t es8388_init(i2c_master_bus_handle_t i2c_bus) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing ES8388 codec (M5Unified sequence)");
    
    /* Add ES8388 device to I2C bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES8388_ADDR,
        .scl_speed_hz = 400000,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &es8388_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ES8388 device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Add PI4IOE5V6408 device to I2C bus */
    i2c_device_config_t pi4io_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PI4IO1_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    ret = i2c_master_bus_add_device(i2c_bus, &pi4io_cfg, &pi4io1_dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add PI4IO1: %s (non-fatal)", esp_err_to_name(ret));
        pi4io1_dev = NULL;
    }
    
    /* Software reset */
    es8388_write_reg(0x00, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    es8388_write_reg(0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    /* === Exact M5Unified ES8388 Init Sequence === */
    /* Format: {length, register, value} - length is always 2 */
    static const uint8_t init_sequence[] = {
        2, 0x00, 0x00,
        2, 0x00, 0x00,
        2, 0x00, 0x0E,
        2, 0x01, 0x00,
        2, 0x02, 0x0A,  /* CHIP POWER: power up all */
        2, 0x03, 0xFF,  /* ADC POWER: power down all */
        2, 0x04, 0x3C,  /* DAC POWER: LOUT1/ROUT1/LOUT2/ROUT2 enable */
        2, 0x05, 0x00,  /* ChipLowPower1 */
        2, 0x06, 0x00,  /* ChipLowPower2 */
        2, 0x07, 0x7C,  /* VSEL */
        2, 0x08, 0x00,  /* I2S slave mode */
        /* ADC config (reg 9-22) skipped - not needed for playback */
        2, 0x17, 0x18,  /* DACCONTROL1: I2S format 16-bit */
        2, 0x18, 0x00,  /* DACCONTROL2: I2S MCLK ratio 128 */
        2, 0x19, 0x20,  /* DACCONTROL3: DAC unmute */
        2, 0x1A, 0x00,  /* LDACVOL */
        2, 0x1B, 0x00,  /* RDACVOL */
        2, 0x1C, 0x08,  /* DACCONTROL6: click free power up/down */
        2, 0x1D, 0x00,  /* DACCONTROL7 */
        2, 0x26, 0x00,  /* DACCONTROL16 */
        2, 0x27, 0xB8,  /* DACCONTROL17: LEFT Ch MIX - routes DAC L to outputs */
        2, 0x2A, 0xB8,  /* DACCONTROL20: RIGHT Ch MIX - routes DAC R to outputs */
        2, 0x2B, 0x08,  /* DACCONTROL21: ADC and DAC separate */
        2, 0x2D, 0x00,  /* DACCONTROL23: VREF output */
        2, 0x2E, 0x10,  /* LOUT1 Volume (50%) */
        2, 0x2F, 0x10,  /* ROUT1 Volume (50%) */
        2, 0x30, 0x10,  /* LOUT2 Volume (50%) */
        2, 0x31, 0x10,  /* ROUT2 Volume (50%) */
        0  /* End marker */
    };
    
    ESP_LOGI(TAG, "Writing ES8388 init sequence...");
    const uint8_t *p = init_sequence;
    while (*p != 0) {
        uint8_t len = *p++;
        if (len == 2) {
            uint8_t reg = *p++;
            uint8_t val = *p++;
            ret = es8388_write_reg(reg, val);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "  Reg 0x%02X = 0x%02X: FAIL (%s)", 
                         reg, val, esp_err_to_name(ret));
            }
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Enable speaker amplifier */
    speaker_enable(1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "ES8388 initialization complete");
    return ESP_OK;
}
