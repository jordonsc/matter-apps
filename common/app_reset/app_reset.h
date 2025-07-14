#pragma once

#include <esp_err.h>

/** 
 * Register callbacks for Factory reset
 *
 * Register factory reset functionality on a button. Creates a static iot_button bound to GPIO configured with
 * CONFIG_RESET_GPIO_PIN.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t register_app_reset(void);
