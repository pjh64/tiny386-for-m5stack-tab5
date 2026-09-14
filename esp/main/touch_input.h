/*
 * Touch Input Driver for M5Stack Tab5
 * Supports GT911 and ST7123/ST7121 touch controllers
 */

#ifndef TOUCH_INPUT_H
#define TOUCH_INPUT_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t touch_input_init(void);
bool touch_input_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif
