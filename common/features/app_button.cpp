#include "app_button.h"

#ifndef CONFIG_BUTTON_COUNT
#define CONFIG_BUTTON_COUNT 0
#endif

#ifndef CONFIG_BUTTON_GPIO_LIST
#define CONFIG_BUTTON_GPIO_LIST ""
#endif

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char* TAG = "app_button";

static uint16_t configured_buttons = 0;
static struct gpio_button button_storage[CONFIG_BUTTON_COUNT];

/**
 * Update the output pin state based on button state
 */
static void update_button_output_pin(gpio_button* button, bool state)
{
    if (button->output_pin != GPIO_NUM_NC) {
        gpio_set_level(button->output_pin, state ? 1 : 0);
        ESP_LOGD(TAG, "Button endpoint %d output pin %d set to %s", 
                 button->endpoint, button->output_pin, state ? "HIGH" : "LOW");
    }
}

/**
 * Unified button event handler for all iot_button events.
 */
static void button_event_handler(void* button_handle, void* usr_data)
{
    gpio_button* button = (gpio_button*)usr_data;
    button_event_t event = iot_button_get_event((button_handle_t)button_handle);
    int switch_endpoint_id = button->endpoint;
    
    switch (event) {
        case BUTTON_PRESS_DOWN:
            ESP_LOGI(TAG, "Button [%d]: DOWN", button->gpio_pin);
            update_button_output_pin(button, true);
            if (button->mechanism == button_mechanism::LATCHING) {
                // Latching switch pressed down (latched)
                chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                    chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, 1);
                    switch_cluster::event::send_switch_latched(switch_endpoint_id, 1);
                });
            } else {
                // Momentary switch pressed down
                chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                    chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, 1);
                    switch_cluster::event::send_initial_press(switch_endpoint_id, 1);
                });
            }
            break;
            
        case BUTTON_PRESS_UP:
            ESP_LOGI(TAG, "Button [%d]: UP", button->gpio_pin);
            update_button_output_pin(button, false);
            if (button->mechanism == button_mechanism::LATCHING) {
                // Latching switch released (unlatched)
                chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                    chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, 0);
                    switch_cluster::event::send_switch_latched(switch_endpoint_id, 0);
                });
            } else {
                // Momentary switch released
                chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                    chip::app::Clusters::Switch::Attributes::CurrentPosition::Set(switch_endpoint_id, 0);
                });
            }
            break;
            
        case BUTTON_PRESS_END:
            ESP_LOGI(TAG, "Button [%d]: PRESS END", button->gpio_pin);
            chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                switch_cluster::event::send_short_release(switch_endpoint_id, 0);
            });
            break;
            
        case BUTTON_SINGLE_CLICK:
            ESP_LOGI(TAG, "Button [%d]: SINGLE-CLICK", button->gpio_pin);
            chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                switch_cluster::event::send_multi_press_complete(switch_endpoint_id, 0, 1);
            });
            break;
            
        case BUTTON_DOUBLE_CLICK:
            ESP_LOGI(TAG, "Button [%d]: DOUBLE-CLICK", button->gpio_pin);
            chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                switch_cluster::event::send_multi_press_complete(switch_endpoint_id, 0, 2);
            });
            break;
            
        case BUTTON_LONG_PRESS_START:
            ESP_LOGI(TAG, "Button [%d]: LONG-PRESS BEGIN", button->gpio_pin);
            chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                switch_cluster::event::send_long_press(switch_endpoint_id, 0);
            });
            break;
            
        case BUTTON_LONG_PRESS_UP:
            ESP_LOGI(TAG, "Button [%d]: LONG-PRESS END", button->gpio_pin);
            chip::DeviceLayer::SystemLayer().ScheduleLambda([switch_endpoint_id]() {
                switch_cluster::event::send_long_release(switch_endpoint_id, 0);
            });
            break;
            
        default:
            // Ignore other button events
            break;
    }
}

/**
 * Create button from GPIO pin number.
 * 
 * This will create an iot_button from a gpio_button and store the handle.
 */
static void button_init(gpio_button* button)
{
    // Initialize button
    const button_config_t btn_cfg = {0};

    // Create button device with the provided GPIO configuration
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = button->gpio_pin,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };

    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &button->button_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create iot_button for GPIO %d: %s", button->gpio_pin, esp_err_to_name(ret));
        return;
    }

    // Register the unified event handler for all relevant events
    if (button->mechanism == button_mechanism::LATCHING) {
        // Latching button events
        iot_button_register_cb(button->button_handle, BUTTON_PRESS_DOWN, NULL, button_event_handler, button);
        iot_button_register_cb(button->button_handle, BUTTON_PRESS_UP, NULL, button_event_handler, button);
    } else {
        // Momentary button events
        iot_button_register_cb(button->button_handle, BUTTON_PRESS_DOWN, NULL, button_event_handler, button);
        iot_button_register_cb(button->button_handle, BUTTON_PRESS_UP, NULL, button_event_handler, button);
        iot_button_register_cb(button->button_handle, BUTTON_PRESS_END, NULL, button_event_handler, button);

        // Multi-press events (always register these for momentary)
        if (button->feature_double_click) {
            iot_button_register_cb(button->button_handle, BUTTON_SINGLE_CLICK, NULL, button_event_handler, button);
            iot_button_register_cb(button->button_handle, BUTTON_DOUBLE_CLICK, NULL, button_event_handler, button);
        }

        // Long-press events
        if (button->feature_long_press) {
            button_event_args_t long_press_args = {
                .long_press = { .press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS }
            };
            iot_button_register_cb(button->button_handle, BUTTON_LONG_PRESS_START, &long_press_args, button_event_handler, button);
            iot_button_register_cb(button->button_handle, BUTTON_LONG_PRESS_UP, &long_press_args, button_event_handler, button);
        }
    }
}


/**
 * Create a button and add it to the Matter node.
 *
 * This function initializes the button driver, creates a new endpoint with a switch cluster,
 * and adds the button to the configured buttons storage.
 *
 * @param[in] node Pointer to the Matter node.
 * @param[in] button Pointer to `gpio_button` structure.
 */
void create_button(node_t* node, gpio_button* button)
{
    ESP_LOGI(TAG, "Creating button on GPIO %d", button->gpio_pin);

    if (configured_buttons >= CONFIG_BUTTON_COUNT) {
        ESP_LOGE(TAG, "Button storage full. Cannot create more buttons. Max: %d", CONFIG_BUTTON_COUNT);
        return;
    }

    // Initialize driver
    button_init(button);
    if (button->button_handle == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize button for GPIO %d", button->gpio_pin);
        return;
    }

    // Create switch config
    generic_switch::config_t switch_config;
    switch_config.switch_cluster.feature_flags = 
         button->mechanism == button_mechanism::LATCHING
        ? cluster::switch_cluster::feature::latching_switch::get_id()
        : cluster::switch_cluster::feature::momentary_switch::get_id();

    // Create switch endpoint with the switch cluster configuration.
    endpoint_t* endpoint = generic_switch::create(node, &switch_config, ENDPOINT_FLAG_NONE, button->button_handle);
    if (endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create switch endpoint for button on GPIO %d", button->gpio_pin);
        iot_button_delete(button->button_handle);
        button->button_handle = nullptr;
        return;
    }

    button->endpoint = endpoint::get_id(endpoint);

    cluster_t* cluster = cluster::get(endpoint, Switch::Id);
    cluster_t* descriptor = cluster::get(endpoint, Descriptor::Id);
    descriptor::feature::taglist::add(descriptor);

    ESP_LOGI(TAG, "Generic Switch created with endpoint_id %d", button->endpoint);

    // Initialize output pin if configured
    if (button->output_pin != GPIO_NUM_NC) {
        gpio_reset_pin(button->output_pin);
        gpio_set_direction(button->output_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(button->output_pin, 0);  // Start with output LOW
        ESP_LOGI(TAG, "Button output pin %d initialised", button->output_pin);
    }

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

    configured_buttons++;
}

/**
 * Destroy a button and clean up its resources.
 * 
 * @param button Pointer to gpio_button structure to destroy.
 */
void destroy_button(gpio_button* button)
{
    if (button && button->button_handle) {
        ESP_LOGI(TAG, "Destroying button on GPIO %d", button->gpio_pin);
        iot_button_delete(button->button_handle);
        button->button_handle = nullptr;
    }
}

/**
 * Using the button list configured via Kconfig, create the buttons and add them to the Matter node.
 */
void create_application_buttons(node_t* node)
{
    // Parse CONFIG_BUTTON_GPIO_LIST for a list of GPIO pins to create buttons.
    // This is a space-separated list of GPIO pin definitions.
    // Example: CONFIG_BUTTON_GPIO_LIST="L9 M9:12 MX8"
    // L9    - latching on pin 9
    // M9    - momentary on pin 9
    // MD9   - momentary-double-click on pin 9
    // ML9   - momentary-long-press on pin 9
    // MX9   - momentary-double-click-long-press on pin 9
    // M9:12 - momentary on pin 9 with output on pin 12

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

            // Parse pin number and optional output pin (separated by ':')
            char* pin_spec = token + idx;
            char* colon_pos = strchr(pin_spec, ':');
            int pin = -1;
            int output_pin = -1;
            
            if (colon_pos != nullptr) {
                // Split at the colon
                *colon_pos = '\0';
                pin = atoi(pin_spec);
                output_pin = atoi(colon_pos + 1);
            } else {
                pin = atoi(pin_spec);
            }
            
            if (pin < 0 || pin >= GPIO_NUM_MAX) {
                ESP_LOGW(TAG, "GPIO pin %d out of range (0-%d) - skipping", pin, GPIO_NUM_MAX - 1);
            } else if (output_pin != -1 && (output_pin < 0 || output_pin >= GPIO_NUM_MAX)) {
                ESP_LOGW(TAG, "Output GPIO pin %d out of range (0-%d) - skipping", output_pin, GPIO_NUM_MAX - 1);
            } else {
                ESP_LOGI(TAG, "Creating button: mechanism=%s, double_click=%d, long_press=%d, pin=%d, output_pin=%s",
                         mechanism == button_mechanism::LATCHING ? "LATCHING" : "MOMENTARY",
                         enable_double_click, enable_long_press, pin,
                         output_pin != -1 ? std::to_string(output_pin).c_str() : "none");

                if (configured_buttons >= CONFIG_BUTTON_COUNT) {
                    ESP_LOGE(TAG, "Button storage full. Cannot create more buttons. Max: %d", CONFIG_BUTTON_COUNT);
                    return;
                }

                // Create the button with the parsed features                
                gpio_button* button = &button_storage[configured_buttons];
                button->gpio_pin = (gpio_num_t)pin;
                button->output_pin = output_pin != -1 ? (gpio_num_t)output_pin : GPIO_NUM_NC;
                button->mechanism = mechanism;
                button->feature_double_click = enable_double_click;
                button->feature_long_press = enable_long_press;

                create_button(node, button);
            }
            token = strtok(nullptr, " ");
        }
    } else {
        ESP_LOGW(TAG, "No GPIO pins configured for buttons. Please set CONFIG_BUTTON_GPIO_LIST.");
    }
}

/**
 * Destroy all application buttons and clean up their resources.
 */
void destroy_application_buttons(void)
{
    ESP_LOGI(TAG, "Destroying %d application buttons", configured_buttons);
    
    for (int i = 0; i < configured_buttons; i++) {
        destroy_button(&button_storage[i]);
    }
    
    configured_buttons = 0;
}
