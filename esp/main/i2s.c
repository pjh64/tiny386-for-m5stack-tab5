#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "es8388.h"
#include "board_m5stack_tab5.h"
#include "common.h"

extern i2c_master_bus_handle_t tab5_get_i2c_bus(void);
extern void mixer_callback(void *opaque, uint8_t *stream, int free);

static void i2s_task(void *arg);

static i2s_chan_handle_t tx_chan = NULL;
static bool audio_active = false;

static StaticTask_t i2s_task_tcb;
static StackType_t *i2s_task_stack = NULL;

void i2s_main(void)
{
    fprintf(stderr, "I2S: Creating I2S task\n");
    
    i2s_task_stack = heap_caps_malloc(2048, MALLOC_CAP_SPIRAM);
    if (!i2s_task_stack) {
        fprintf(stderr, "I2S: Failed to allocate task stack in PSRAM\n");
        return;
    }
    
    xTaskCreateStaticPinnedToCore(
        i2s_task,
        "i2s_task",
        2048,
        NULL,
        5,
        i2s_task_stack,
        &i2s_task_tcb,
        0
    );
}

static void i2s_task(void *arg)
{
    fprintf(stderr, "I2S: Task running on core %d (priority 5)\n", esp_cpu_get_core_id());
    
    xEventGroupWaitBits(global_event_group, BIT0, pdFALSE, pdFALSE, portMAX_DELAY);
    fprintf(stderr, "I2S: Emulator ready\n");
    
    i2c_master_bus_handle_t bus = NULL;
    int retries = 0;
    while (bus == NULL && retries < 100) {
        bus = tab5_get_i2c_bus();
        if (bus == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            retries++;
        }
    }
    
    if (bus == NULL) {
        fprintf(stderr, "I2S: ERROR - I2C bus not available!\n");
        vTaskDelete(NULL);
        return;
    }
    fprintf(stderr, "I2S: Got I2C bus after %d retries\n", retries);
    
    esp_err_t codec_ret = es8388_init(bus);
    if (codec_ret != ESP_OK) {
        fprintf(stderr, "I2S: WARNING - ES8388 init failed: %s\n", esp_err_to_name(codec_ret));
    } else {
        fprintf(stderr, "I2S: ES8388 initialized\n");
        es8388_mute(1);
        speaker_enable(0);
    }
    
    /* Audio buffer in PSRAM */
    int16_t *buf = heap_caps_malloc(MIXER_BUF_LEN * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (buf == NULL) {
        fprintf(stderr, "I2S: Failed to allocate mixer buffer\n");
        vTaskDelete(NULL);
        return;
    }
    
    /* ABSOLUTE MINIMUM DMA: 2x64 frames = 512 bytes total
     * This is the smallest config that still produces audio */
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    tx_chan_cfg.auto_clear = true;
    tx_chan_cfg.dma_desc_num = 2;      /* Absolute minimum */
    tx_chan_cfg.dma_frame_num = 64;    /* Absolute minimum */
    
    int chan_retries = 0;
    while (chan_retries < 60) {
        if (i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL) == ESP_OK) {
            fprintf(stderr, "I2S: Channel created (2x64 frames = 512B DMA)\n");
            break;
        }
        chan_retries++;
        fprintf(stderr, "I2S: DMA alloc failed, retry %d/60...\n", chan_retries);
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Longer wait between retries */
    }
    
    if (tx_chan == NULL) {
        fprintf(stderr, "I2S: Failed to create channel\n");
        heap_caps_free(buf);
        vTaskDelete(NULL);
        return;
    }

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK,
            .ws   = I2S_WS,
            .dout = I2S_DOUT,
            .din  = -1,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    int init_retries = 0;
    while (init_retries < 60) {
        if (i2s_channel_init_std_mode(tx_chan, &tx_std_cfg) == ESP_OK) {
            fprintf(stderr, "I2S: Channel initialized\n");
            break;
        }
        init_retries++;
        fprintf(stderr, "I2S: Channel init failed, retry %d/60...\n", init_retries);
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Longer wait between retries */
    }
    
    if (init_retries >= 60) {
        fprintf(stderr, "I2S: Failed to init channel after 60 retries\n");
        i2s_del_channel(tx_chan);
        heap_caps_free(buf);
        vTaskDelete(NULL);
        return;
    }

    i2s_channel_enable(tx_chan);
    fprintf(stderr, "I2S: Starting audio loop\n");

    int loop_count = 0;
    for (;;) {
        size_t bwritten;
        memset(buf, 0, MIXER_BUF_LEN * sizeof(int16_t));
        mixer_callback(globals.pc, (uint8_t *) buf, MIXER_BUF_LEN * sizeof(int16_t));
        
        int32_t peak = 0;
        for (int i = 0; i < MIXER_BUF_LEN; i++) {
            int32_t s = buf[i];
            if (s < 0) s = -s;
            if (s > peak) peak = s;
        }
        
        if (peak > 100 && !audio_active) {
            fprintf(stderr, "I2S: Audio detected (peak=%" PRId32 "), enabling codec\n", peak);
            speaker_enable(1);
            es8388_mute(0);
            audio_active = true;
        }
        
        i2s_channel_write(tx_chan, buf, MIXER_BUF_LEN * sizeof(int16_t), &bwritten, portMAX_DELAY);
        
        if (++loop_count % 100 == 0) {
            fprintf(stderr, "I2S: Loop %d, peak=%" PRId32 ", active=%d\n", 
                    loop_count, peak, audio_active);
        }
    }
}
