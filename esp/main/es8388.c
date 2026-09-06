#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "es8388.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ES8388_ADDR 0x10
#define TAG "ES8388"

// ES8388 Register Map (korrigiert)
#define ES8388_CONTROL1         0x00
#define ES8388_CONTROL2         0x01
#define ES8388_CHIPPOWER        0x02
#define ES8388_ADCPOWER         0x03
#define ES8388_DACPOWER         0x04
#define ES8388_CHIPLOPOW1       0x05
#define ES8388_CHIPLOPOW2       0x06
#define ES8388_ANAVOLMANAG      0x07
#define ES8388_MASTERMODE       0x08
#define ES8388_DACCONTROL1      0x17
#define ES8388_DACCONTROL2      0x18
#define ES8388_DACCONTROL3      0x19
#define ES8388_DACCONTROL4      0x1A
#define ES8388_DACCONTROL5      0x1B
#define ES8388_DACCONTROL6      0x1C
#define ES8388_DACCONTROL7      0x1D
#define ES8388_DACCONTROL8      0x1E
#define ES8388_DACCONTROL9      0x1F
#define ES8388_DACCONTROL10     0x20
#define ES8388_DACCONTROL11     0x21
#define ES8388_DACCONTROL12     0x22
#define ES8388_DACCONTROL13     0x23
#define ES8388_DACCONTROL14     0x24
#define ES8388_DACCONTROL15     0x25
#define ES8388_DACCONTROL16     0x26
#define ES8388_DACCONTROL17     0x27
#define ES8388_LOUT1_VOL        0x2E
#define ES8388_ROUT1_VOL        0x2F
#define ES8388_LOUT2_VOL        0x30
#define ES8388_ROUT2_VOL        0x31

static i2c_master_dev_handle_t es8388_dev = NULL;

static esp_err_t es8388_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(es8388_dev, write_buf, 2, 100);
}

static esp_err_t es8388_read_reg(uint8_t reg_addr, uint8_t *data) {
    return i2c_master_transmit_receive(es8388_dev, &reg_addr, 1, data, 1, 100);
}

static void es8388_reset(void) {
    ESP_LOGI(TAG, "Performing ES8388 reset sequence");
    
    // Mute DAC outputs first
    es8388_write_reg(ES8388_DACCONTROL3, 0x06);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Power down DAC
    es8388_write_reg(ES8388_DACPOWER, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Reset the codec
    es8388_write_reg(ES8388_CONTROL1, 0x80);  // Software reset
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clear reset
    es8388_write_reg(ES8388_CONTROL1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "ES8388 reset complete");
}

esp_err_t es8388_init(i2c_master_bus_handle_t i2c_bus) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing ES8388 codec");
    
    // Add device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES8388_ADDR,
        .scl_speed_hz = 100000,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &es8388_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ES8388 device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Perform reset sequence
    es8388_reset();
    
    // Check if device is present
    uint8_t reg_val;
    ret = es8388_read_reg(ES8388_CONTROL1, &reg_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ES8388 CONTROL1 register");
        return ret;
    }
    
    ESP_LOGI(TAG, "ES8388 CONTROL1 = 0x%02X", reg_val);
    
    // Configure ES8388 for playback
    // Set master mode
    es8388_write_reg(ES8388_MASTERMODE, 0x00);
    
    // Power up DAC
    es8388_write_reg(ES8388_DACPOWER, 0xC0);  // Enable DAC L/R
    
    // Configure DAC format: 16-bit, I2S
    es8388_write_reg(ES8388_DACCONTROL1, 0x18);  // 16-bit I2S
    es8388_write_reg(ES8388_DACCONTROL2, 0x02);  // DAC Mute off, De-emphasis off
    
    // Set DAC volume to maximum
    es8388_write_reg(ES8388_DACCONTROL4, 0x00);  // Left DAC volume
    es8388_write_reg(ES8388_DACCONTROL5, 0x00);  // Right DAC volume
    
    // Set output volume
    es8388_write_reg(ES8388_LOUT1_VOL, 0x1E);  // LOUT1 volume
    es8388_write_reg(ES8388_ROUT1_VOL, 0x1E);  // ROUT1 volume
    
    // Unmute DAC outputs
    es8388_write_reg(ES8388_DACCONTROL3, 0x00);
    
    ESP_LOGI(TAG, "ES8388 initialization complete");
    
    return ESP_OK;
}
