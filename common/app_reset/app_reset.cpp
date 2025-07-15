#include <esp_log.h>
#include <esp_matter.h>
#include "iot_button.h"
#include "button_gpio.h"

#ifndef CONFIG_RESET_GPIO_PIN
#define CONFIG_RESET_GPIO_PIN 0
#endif

#ifndef CONFIG_RESET_HOLD_TIME
#define CONFIG_RESET_HOLD_TIME 3000
#endif

static const char *TAG = "app_reset";
static button_handle_t reset_button_handle = NULL;


static void factory_restart_begin(void* arg, void* data)
{
    ESP_LOGI(TAG, "Starting factory reset");
    esp_matter::factory_reset();
}

esp_err_t register_app_reset()
{
    if (reset_button_handle != NULL) {
        ESP_LOGW(TAG, "Reset button already registered");
        return ESP_ERR_INVALID_STATE;
    }

    // Create button configuration
    button_config_t button_cfg = {
        .long_press_time = CONFIG_RESET_HOLD_TIME
    };

    // Create GPIO button configuration for CONFIG_RESET_GPIO_PIN
    button_gpio_config_t gpio_cfg = {
        .gpio_num = CONFIG_RESET_GPIO_PIN,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };

    // Create the GPIO button device
    esp_err_t err = iot_button_new_gpio_device(&button_cfg, &gpio_cfg, &reset_button_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create GPIO button device: %s", esp_err_to_name(err));
        return err;
    }

    // Register callbacks
    err = iot_button_register_cb(reset_button_handle, BUTTON_LONG_PRESS_UP, NULL, factory_restart_begin, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register button callbacks: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Factory reset button registered on GPIO %d", CONFIG_RESET_GPIO_PIN);
    return ESP_OK;
}
