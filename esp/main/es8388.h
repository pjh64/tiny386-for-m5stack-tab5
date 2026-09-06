#ifndef ES8388_H
#define ES8388_H

#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t es8388_init(i2c_master_bus_handle_t i2c_bus);

#endif /* ES8388_H */
