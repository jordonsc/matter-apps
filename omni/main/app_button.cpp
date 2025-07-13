#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <app-common/zap-generated/attributes/Accessors.h>

#include <app_button.h>
#include <iot_button.h>
#include <button_gpio.h>

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char *TAG = "app_button";

static uint16_t configured_buttons = 0;
static button_endpoint button_list[CONFIG_BUTTON_COUNT];
static struct gpio_button button_storage[CONFIG_BUTTON_COUNT];


esp_err_t button_attribute_update(btn_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
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
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
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
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
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
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: UP", button->GPIO_PIN_VALUE);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Device the endpoint position to 0 (released)
        chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, pos);
    });
}

/**
 * Button press end.
 */
static void button_on_press_end(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: PRESS END", button->GPIO_PIN_VALUE);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send short release event
        switch_cluster::event::send_short_release(switch_endpoint_id, pos);
    });
}

/**
 * Button single-click.
 */
static void button_on_single_click(void *arg, void *data)
{
    gpio_button * button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
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
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
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
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
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
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
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
btn_handle_t button_init(gpio_button* button)
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
    // Standard momentary button callbacks
    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, button_on_down, button);
    iot_button_register_cb(handle, BUTTON_PRESS_UP, NULL, button_on_up, button);
    iot_button_register_cb(handle, BUTTON_PRESS_END, NULL, button_on_press_end, button);

    // Multi-press callbacks
    iot_button_register_cb(handle, BUTTON_SINGLE_CLICK, NULL, button_on_single_click, button);
    iot_button_register_cb(handle, BUTTON_DOUBLE_CLICK, NULL, button_on_double_click, button);

    // Long-press callbacks
    button_event_args_t long_press_args = {
        .long_press = { .press_time = CONFIG_BUTTON_LONG_PRESS_TIME }
    };
    iot_button_register_cb(handle, BUTTON_LONG_PRESS_START, &long_press_args, button_on_long_press_start, button);
    iot_button_register_cb(handle, BUTTON_LONG_PRESS_UP, &long_press_args, button_on_long_press_end, button);
#endif

    return (btn_handle_t)handle;
}


/**
 * Create a button and add it to the Matter node.
 *
 * This function initializes the button driver, creates a new endpoint with a switch cluster,
 * and adds the button to the list of configured buttons.
 *
 * @param[in] button Pointer to `gpio_button` structure.
 * @param[in] node Pointer to the Matter node.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_NO_MEM if button storage exceeded.
 * @return ESP_FAIL if node or endpoint creation fails.
 */
esp_err_t create_button(struct gpio_button* button, node_t* node)
{
    if (configured_buttons >= CONFIG_BUTTON_COUNT) {
        ESP_LOGE(TAG, "Button storage full. Cannot create more buttons. Max: %d", CONFIG_BUTTON_COUNT);
        return ESP_ERR_NO_MEM;
    }

    // Initialize driver
    btn_handle_t button_handle = button_init(button);

    // Create switch config
    generic_switch::config_t switch_config;
    switch_config.switch_cluster.feature_flags = 
#if CONFIG_GENERIC_SWITCH_TYPE_LATCHING
    cluster::switch_cluster::feature::latching_switch::get_id();
#endif
#if CONFIG_GENERIC_SWITCH_TYPE_MOMENTARY
    cluster::switch_cluster::feature::momentary_switch::get_id();
#endif

    // Create switch endpoint with the switch cluster configuration.
    endpoint_t *endpoint = generic_switch::create(node, &switch_config, ENDPOINT_FLAG_NONE, button_handle);
    static uint16_t generic_switch_endpoint_id = endpoint::get_id(endpoint);
    
    cluster_t* descriptor = cluster::get(endpoint, Descriptor::Id);
    descriptor::feature::taglist::add(descriptor);

    ESP_LOGI(TAG, "Generic Switch created with endpoint_id %d", generic_switch_endpoint_id);

    // These node and endpoint handles can be used to create/add other endpoints and clusters
    if (!node || !endpoint)
    {
        ESP_LOGE(TAG, "Matter node creation failed");
        return ESP_FAIL;
    }

    // Save the button and endpoint in the button storage
    button_list[configured_buttons].button = button;
    button_list[configured_buttons].endpoint = endpoint::get_id(endpoint);
    ++configured_buttons;

    /* Add additional features to the node */
    cluster_t *cluster = cluster::get(endpoint, Switch::Id);

#if CONFIG_GENERIC_SWITCH_TYPE_MOMENTARY
    cluster::switch_cluster::feature::action_switch::add(cluster);
    
#if CONFIG_BUTTON_ENABLE_DOUBLE_CLICK
    // Configure multi-press feature for momentary switches
    ESP_LOGI(TAG, "Enabling double-click feature for momentary switches");
    cluster::switch_cluster::feature::momentary_switch_multi_press::config_t msm;
    msm.multi_press_max = 2;
    cluster::switch_cluster::feature::momentary_switch_multi_press::add(cluster, &msm);
#endif

#if CONFIG_BUTTON_ENABLE_LONG_PRESS
    // Configure long-press feature for momentary switches
    ESP_LOGI(TAG, "Enabling long-press feature for momentary switches");
    cluster::switch_cluster::feature::momentary_switch_long_press::add(cluster);
#endif

#endif

    return ESP_OK;
}

/** 
 * Create button from GPIO pin number.
 *
 * This function creates a gpio_button from an integer pin number and calls create_button.
 * It uses static storage to prevent memory issues with callback references.
 *
 * @param[in] pin_number GPIO pin number as integer
 * @param[in] node Matter node pointer
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_NO_MEM if button storage exceeded.
 * @return ESP_FAIL if node or endpoint creation fails.
 */
esp_err_t create_button(int pin_number, node_t* node)
{
    if (configured_buttons >= CONFIG_BUTTON_COUNT) {
        ESP_LOGE(TAG, "Button storage full. Cannot create more buttons. Max: %d", CONFIG_BUTTON_COUNT);
        return ESP_ERR_NO_MEM;
    }
    
    button_storage[configured_buttons].GPIO_PIN_VALUE = (gpio_num_t)pin_number;
    return create_button(&button_storage[configured_buttons], node);
}

/**
 * Get the endpoint ID for a given button.
 *
 * This function searches through the configured buttons to find the endpoint ID associated with the button.
 *
 * @param[in] button Pointer to `gpio_button`.
 *
 * @return Endpoint ID if found, -1 if not found.
 */
int get_button_endpoint(gpio_button* button)
{
    for (int i = 0; i < configured_buttons; i++) {
        if (button_list[i].button == button) {
            return button_list[i].endpoint;
        }
    }
    return -1;
}

/**
 * Using the button list configured via Kconfig, create the buttons and add them to the Matter node.
 */
void create_application_buttons(node_t *node)
{
    // Parse CONFIG_BUTTON_GPIO_LIST for a list of GPIO pins to create buttons.
    // This is a comma-separated list of GPIO pin numbers.
    // Example: CONFIG_BUTTON_GPIO_LIST="9,10,11"
    const char* gpio_list_str = CONFIG_BUTTON_GPIO_LIST;
    if (gpio_list_str && gpio_list_str[0] != '\0') {
        char buf[128];
        strncpy(buf, gpio_list_str, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* token = strtok(buf, ",");
        while (token != nullptr) {
            char* end;
            long pin = strtol(token, &end, 10);
            
            // Check for conversion errors
            if (end == token) {
                ESP_LOGW(TAG, "Invalid GPIO pin format: '%s' - skipping", token);
            } else if (pin < 0 || pin >= GPIO_NUM_MAX) {
                ESP_LOGW(TAG, "GPIO pin %ld out of range (0-%d) - skipping", pin, GPIO_NUM_MAX - 1);
            } else {
                ESP_LOGI(TAG, "Creating button for GPIO pin: %ld", pin);
                create_button((int)pin, node);
            }
            token = strtok(nullptr, ",");
        }
    } else {
        ESP_LOGW(TAG, "No GPIO pins configured for buttons. Please set CONFIG_BUTTON_GPIO_LIST.");
    }
}
