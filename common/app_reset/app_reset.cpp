#include <esp_log.h>
#include <esp_matter.h>
#include "iot_button.h"
#include "button_gpio.h"

static const char *TAG = "app_reset";
static bool perform_factory_reset = false;
static button_handle_t reset_button_handle = NULL;


static void factory_restart_begin(void* arg, void* data)
{
    if (perform_factory_reset) {
        ESP_LOGI(TAG, "Starting factory reset");
        esp_matter::factory_reset();
        perform_factory_reset = false;
    }
}

esp_err_t register_app_reset()
{
    esp_err_t err = ESP_OK;

    // Create button configuration
    button_config_t button_cfg = {
        .long_press_time = CONFIG_RESET_HOLD_TIME
    };

    // Create GPIO button configuration for GPIO 0
    button_gpio_config_t gpio_cfg = {
        .gpio_num = atoi(CONFIG_RESET_GPIO_PIN),
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };

    // Create the GPIO button device
    err = iot_button_new_gpio_device(&button_cfg, &gpio_cfg, &reset_button_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create GPIO button device: %s", esp_err_to_name(err));
        return err;
    }

    // Register callbacks
    err |= iot_button_register_cb(reset_button_handle, BUTTON_LONG_PRESS_UP, NULL, factory_restart_begin, NULL);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register button callbacks: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Factory reset button registered on GPIO 0");
    return ESP_OK;
}
