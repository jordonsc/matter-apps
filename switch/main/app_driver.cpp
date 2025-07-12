#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <app-common/zap-generated/attributes/Accessors.h>

#include <app_priv.h>
#include <iot_button.h>
#include <button_gpio.h>

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

static const char *TAG = "app_driver";

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val) 
{
    return ESP_OK;
}

#if CONFIG_GENERIC_SWITCH_TYPE_LATCHING
static uint8_t latching_switch_previous_position = 0;
static void app_driver_button_switch_latched(void *arg, void *data)
{
    ESP_LOGI(TAG, "Switch lached pressed");
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_endpoint(button) : 1;
    // Press moves Position from 0 (idle) to 1 (press) and vice versa
    uint8_t newPosition = (latching_switch_previous_position == 1) ? 0 : 1;
    latching_switch_previous_position = newPosition;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, newPosition]() {
        chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, newPosition);
        // SwitchLatched event takes newPosition as event data
        switch_cluster::event::send_switch_latched(switch_endpoint_id, newPosition);
    });
}
#endif

#if CONFIG_GENERIC_SWITCH_TYPE_MOMENTARY

/**
 * Button depressed.
 */
static void button_on_down(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: DOWN", button->GPIO_PIN_VALUE);

    uint8_t pos = 1;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Device the endpoint position to 1 (pressed)
        chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, pos);

        // Send initial press event
        switch_cluster::event::send_initial_press(switch_endpoint_id, pos);
    });
}

/**
 * Button released.
 */
static void button_on_up(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: UP", button->GPIO_PIN_VALUE);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Device the endpoint position to 0 (released)
        chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, pos);
    });
}

/**
 * Button single-click.
 */
static void button_on_single_click(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: SINGLE-CLICK", button->GPIO_PIN_VALUE);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send single-click event
        switch_cluster::event::send_multi_press_complete(switch_endpoint_id, pos, 1);
    });
}

/**
 * Button double-click.
 */
static void button_on_double_click(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: DOUBLE-CLICK", button->GPIO_PIN_VALUE);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send double-click event
        switch_cluster::event::send_multi_press_complete(switch_endpoint_id, pos, 2);
    });
}

/**
 * Button long-press start.
 */
static void button_on_long_press_start(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: LONG-PRESS BEGIN", button->GPIO_PIN_VALUE);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send long-press start event
        switch_cluster::event::send_long_press(switch_endpoint_id, pos);
    });
}

/**
 * Button long-press end.
 */
static void button_on_long_press_end(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: LONG-PRESS END", button->GPIO_PIN_VALUE);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send long-press start event
        switch_cluster::event::send_long_release(switch_endpoint_id, pos);
    });
}

#endif

/**
 * Create button from GPIO pin number.
 * 
 * This will create an iot_button from a gpio_button.
 */
app_driver_handle_t app_driver_button_init(gpio_button* button)
{
    // Initialize button
    button_handle_t handle = NULL;
    const button_config_t btn_cfg = {0};

    // Create button device with the provided GPIO configuration
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = button->GPIO_PIN_VALUE,
        .active_level = 0,
    };

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device");
        return NULL;
    }


#if CONFIG_GENERIC_SWITCH_TYPE_LATCHING
    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_switch_latched, button);
#endif

#if CONFIG_GENERIC_SWITCH_TYPE_MOMENTARY
    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, button_on_down, button);
    iot_button_register_cb(handle, BUTTON_PRESS_UP, NULL, button_on_up, button);
    iot_button_register_cb(handle, BUTTON_SINGLE_CLICK, NULL, button_on_single_click, button);
    iot_button_register_cb(handle, BUTTON_DOUBLE_CLICK, NULL, button_on_double_click, button);

    button_event_args_t long_press_args = {
        .long_press = { .press_time = CONFIG_BUTTON_LONG_PRESS_TIME }
    };

    iot_button_register_cb(handle, BUTTON_LONG_PRESS_START, &long_press_args, button_on_long_press_start, button);
    iot_button_register_cb(handle, BUTTON_LONG_PRESS_UP, &long_press_args, button_on_long_press_end, button);

#endif

    return (app_driver_handle_t)handle;
}
