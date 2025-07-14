#include "app_button.h"

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char* TAG = "app_button";

static uint16_t configured_buttons = 0;
static button_endpoint button_list[CONFIG_BUTTON_COUNT];
static struct gpio_button button_storage[CONFIG_BUTTON_COUNT];


/**
 * Latching switched changed position.
 */
static void button_switch_latched(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: LATCHED", button->gpio_pin);

    uint8_t pos = (uint8_t)switch_latch_position::OPEN;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, pos);
        
        // SwitchLatched event takes position as event data
        switch_cluster::event::send_switch_latched(switch_endpoint_id, pos);
    });
}

/**
 * Latching switched changed position.
 */
static void button_switch_unlatched(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: UNLATCHED", button->gpio_pin);

    uint8_t pos = (uint8_t)switch_latch_position::CLOSED;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, pos);
        
        // SwitchLatched event takes position as event data
        switch_cluster::event::send_switch_latched(switch_endpoint_id, pos);
    });
}

/**
 * Button initial depressed.
 */
static void button_on_down(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: DOWN", button->gpio_pin);

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
static void button_on_up(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: UP", button->gpio_pin);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Device the endpoint position to 0 (released)
        chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, pos);
    });
}

/**
 * Button press end.
 */
static void button_on_press_end(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: PRESS END", button->gpio_pin);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send short release event
        switch_cluster::event::send_short_release(switch_endpoint_id, pos);
    });
}

/**
 * Button single-click.
 */
static void button_on_single_click(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: SINGLE-CLICK", button->gpio_pin);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send single-click event
        switch_cluster::event::send_multi_press_complete(switch_endpoint_id, pos, 1);
    });
}

/**
 * Button double-click.
 */
static void button_on_double_click(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: DOUBLE-CLICK", button->gpio_pin);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send double-click event
        switch_cluster::event::send_multi_press_complete(switch_endpoint_id, pos, 2);
    });
}

/**
 * Button long-press start.
 */
static void button_on_long_press_start(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: LONG-PRESS BEGIN", button->gpio_pin);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send long-press start event
        switch_cluster::event::send_long_press(switch_endpoint_id, pos);
    });
}

/**
 * Button long-press end.
 */
static void button_on_long_press_end(void* arg, void* data)
{
    gpio_button* button = (gpio_button*)data;
    int switch_endpoint_id = (button != NULL) ? get_button_endpoint(button) : 1;
    ESP_LOGI(TAG, "Button [%d]: LONG-PRESS END", button->gpio_pin);

    uint8_t pos = 0;
    chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id, pos]() {
        // Send long-press start event
        switch_cluster::event::send_long_release(switch_endpoint_id, pos);
    });
}

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
        .gpio_num = button->gpio_pin,
        .active_level = 0,
    };

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device");
        return NULL;
    }

    if (button->mechanism == button_mechanism::LATCHING) {
        // Latching button callbacks
        iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, button_switch_latched, button);
        iot_button_register_cb(handle, BUTTON_PRESS_UP, NULL, button_switch_unlatched, button);
    }

    if (button->mechanism == button_mechanism::MOMENTARY) {
        // Standard momentary button callbacks
        iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, button_on_down, button);
        iot_button_register_cb(handle, BUTTON_PRESS_UP, NULL, button_on_up, button);
        iot_button_register_cb(handle, BUTTON_PRESS_END, NULL, button_on_press_end, button);

        // Multi-press callbacks
        iot_button_register_cb(handle, BUTTON_SINGLE_CLICK, NULL, button_on_single_click, button);
        iot_button_register_cb(handle, BUTTON_DOUBLE_CLICK, NULL, button_on_double_click, button);

        // Long-press callbacks
        button_event_args_t long_press_args = {
            .long_press = { .press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS }
        };
        iot_button_register_cb(handle, BUTTON_LONG_PRESS_START, &long_press_args, button_on_long_press_start, button);
        iot_button_register_cb(handle, BUTTON_LONG_PRESS_UP, &long_press_args, button_on_long_press_end, button);
    }

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
esp_err_t create_button(node_t* node, struct gpio_button* button)
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
         button->mechanism == button_mechanism::LATCHING
        ? cluster::switch_cluster::feature::latching_switch::get_id()
        : cluster::switch_cluster::feature::momentary_switch::get_id();

    // Create switch endpoint with the switch cluster configuration.
    endpoint_t* endpoint = generic_switch::create(node, &switch_config, ENDPOINT_FLAG_NONE, button_handle);
    if (endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create switch endpoint for button on GPIO %d", button->gpio_pin);
        return ESP_FAIL;
    }

    cluster_t* cluster = cluster::get(endpoint, Switch::Id);
    cluster_t* descriptor = cluster::get(endpoint, Descriptor::Id);
    descriptor::feature::taglist::add(descriptor);

    ESP_LOGI(TAG, "Generic Switch created with endpoint_id %d", endpoint::get_id(endpoint));

    // Save the button and endpoint in the button storage
    button_list[configured_buttons].button = button;
    button_list[configured_buttons].endpoint = endpoint::get_id(endpoint);
    ++configured_buttons;

    // Add additional features to the endpoint
    if (button->mechanism == button_mechanism::MOMENTARY) {
        // For momentary switches, we will use the newer 'action switch' feature
        cluster::switch_cluster::feature::action_switch::add(cluster);
    
        if (button->feature_double_click) {
            // Configure multi-press feature for momentary switches
            // Note that we limit this to 2 presses for simplicity
            ESP_LOGI(TAG, "Enabling double-click feature for momentary switches");
            cluster::switch_cluster::feature::momentary_switch_multi_press::config_t msm;
            msm.multi_press_max = 2;
            cluster::switch_cluster::feature::momentary_switch_multi_press::add(cluster, &msm);
        }

        if (button->feature_long_press) {
            // Configure long-press feature for momentary switches
            ESP_LOGI(TAG, "Enabling long-press feature for momentary switches");
            cluster::switch_cluster::feature::momentary_switch_long_press::add(cluster);
        }
    } else if (button->mechanism == button_mechanism::LATCHING) {
        // Latching switches (mutally exclusive with momentary)
        cluster::switch_cluster::feature::latching_switch::add(cluster);
    }

    return ESP_OK;
}

/**
 * Creates and configures a new button instance.
 *
 * This function initializes a button with the specified GPIO pin, mechanism type,
 * double-click and long-press capabilities, and associates it with a node.
 * 
 * It checks if the button storage is full before proceeding.
 *
 * @param pin_number      GPIO pin number to which the button is connected.
 * @param mechanism       The button mechanism type (momentary vs latching).
 * @param dbl             Enable double-click detection if true.
 * @param lpress          Enable long-press detection if true.
 * @param node            Pointer to the node to associate with the button.
 * @return esp_err_t      ESP_OK on success, ESP_ERR_NO_MEM if storage is full, or other error codes.
 */
esp_err_t create_button(node_t* node, int pin_number, button_mechanism mechanism, bool dbl, bool lpress)
{
    if (configured_buttons >= CONFIG_BUTTON_COUNT) {
        ESP_LOGE(TAG, "Button storage full. Cannot create more buttons. Max: %d", CONFIG_BUTTON_COUNT);
        return ESP_ERR_NO_MEM;
    }

    gpio_button* btn = &button_storage[configured_buttons];
    btn->gpio_pin = (gpio_num_t)pin_number;
    btn->mechanism = mechanism;
    btn->feature_double_click = dbl;
    btn->feature_long_press = lpress;

    return create_button(node, btn);
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
    // FIXME: move the endpoint_id to the gpio_button struct - this is pointless
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
void create_application_buttons(node_t* node)
{
    // Parse CONFIG_BUTTON_GPIO_LIST for a list of GPIO pins to create buttons.
    // This is a space-separated list of GPIO pin definitions.
    // Example: CONFIG_BUTTON_GPIO_LIST="L9 M9 MX8"
    // L9  - latching on pin 9
    // M9  - momentary on pin 9
    // MD9 - momentary-double-click on pin 9
    // ML9 - momentary-long-press on pin 9
    // MX9 - momentary-double-click-long-press on pin 9

    const char* gpio_list_str = CONFIG_BUTTON_GPIO_LIST;
    if (gpio_list_str && gpio_list_str[0] != '\0') {
        char buf[128];
        strncpy(buf, gpio_list_str, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* token = strtok(buf, " ");
        while (token != nullptr) {
            size_t len = strlen(token);
            if (len < 2) {
                ESP_LOGW(TAG, "Invalid button definition: '%s' - skipping", token);
                token = strtok(nullptr, " ");
                continue;
            }

            // Parse mechanism and features
            button_mechanism mechanism = button_mechanism::MOMENTARY;
            bool enable_double_click = false;
            bool enable_long_press = false;

            int idx = 0;
            if (token[idx] == 'L') {
                mechanism = button_mechanism::LATCHING;
                idx++;
            } else if (token[idx] == 'M') {
                mechanism = button_mechanism::MOMENTARY;
                idx++;
            } else {
                ESP_LOGW(TAG, "Unknown mechanism in button definition: '%s' - skipping", token);
                token = strtok(nullptr, " ");
                continue;
            }

            // Parse features
            while (idx < len && (token[idx] == 'D' || token[idx] == 'L' || token[idx] == 'X')) {
                if (token[idx] == 'D') {
                    enable_double_click = true;
                } else if (token[idx] == 'L') {
                    enable_long_press = true;
                } else if (token[idx] == 'X') {
                    enable_double_click = true;
                    enable_long_press = true;
                }
                idx++;
            }

            // Parse pin number
            int pin = atoi(token + idx);
            if (pin < 0 || pin >= GPIO_NUM_MAX) {
                ESP_LOGW(TAG, "GPIO pin %d out of range (0-%d) - skipping", pin, GPIO_NUM_MAX - 1);
            } else {
                ESP_LOGI(TAG, "Creating button: mechanism=%s, double_click=%d, long_press=%d, pin=%d",
                         mechanism == button_mechanism::LATCHING ? "LATCHING" : "MOMENTARY",
                         enable_double_click, enable_long_press, pin);

                // Create the button with the parsed features
                create_button(node, pin, mechanism, enable_double_click, enable_long_press);
            }
            token = strtok(nullptr, " ");
        }
    } else {
        ESP_LOGW(TAG, "No GPIO pins configured for buttons. Please set CONFIG_BUTTON_GPIO_LIST.");
    }
}
