#include "app_onoff.h"

#ifndef CONFIG_ONOFF_COUNT
#define CONFIG_ONOFF_COUNT 0
#endif

#ifndef CONFIG_ONOFF_GPIO_LIST
#define CONFIG_ONOFF_GPIO_LIST ""
#endif

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char *TAG = "app_onoff";

static uint16_t configured_onoff_devices = 0;
static struct gpio_onoff onoff_storage[CONFIG_ONOFF_COUNT];

/**
 * Update the output pin state if configured
 */
static void update_onoff_output_pin(gpio_onoff* onoff)
{
    if (onoff->output_pin != GPIO_NUM_NC) {
        gpio_set_level(onoff->output_pin, onoff->state ? 1 : 0);
        ESP_LOGD(TAG, "OnOff endpoint %d output pin %d set to %s", 
                 onoff->endpoint, onoff->output_pin, onoff->state ? "HIGH" : "LOW");
    }
}

/**
 * Handler for iot_button events.
 */
static void onoff_button_handler(void* button_handle, void* usr_data)
{
    gpio_onoff* onoff = (gpio_onoff*)usr_data;
    button_event_t event = iot_button_get_event((button_handle_t)button_handle);
    
    // Only respond to button press down events for toggle behavior
    if (event != BUTTON_PRESS_DOWN) {
        return;
    }

    // Toggle the state
    bool new_state = !onoff->state;

    // Update the state
    onoff->state = new_state;
    
    ESP_LOGI(
        TAG, "OnOff endpoint %d on GPIO %d toggled to %s", 
        onoff->endpoint, onoff->gpio_pin, 
        onoff->state ? "ON" : "OFF"
    );

    // Update output pin if configured
    update_onoff_output_pin(onoff);

    esp_matter_attr_val_t val = esp_matter_bool(onoff->state);
    attribute::update(
        onoff->endpoint,
        OnOff::Id,
        OnOff::Attributes::OnOff::Id, 
        &val
    );
}

/**
 * Create an on/off device from a GPIO pin.
 * 
 * This implementation creates a generic on/off light endpoint and uses the iot_button component 
 * for GPIO handling with debouncing.
 *
 * @param node Pointer to the Matter node.
 * @param onoff Pointer to `gpio_onoff` structure containing device configuration.
 */
void create_onoff_device(node_t* node, gpio_onoff* onoff)
{
    endpoint_t* onoff_endpoint = nullptr;
    
    // Create the appropriate endpoint based on device type
    switch (onoff->type) {
        case onoff_type::LIGHT: {
            ESP_LOGI(TAG, "Creating on/off <light> on GPIO %d", onoff->gpio_pin);
            on_off_light::config_t config;
            onoff_endpoint = on_off_light::create(node, &config, ENDPOINT_FLAG_NONE, onoff);
            break;
        }
        case onoff_type::OUTLET: {
            ESP_LOGI(TAG, "Creating on/off <outlet> on GPIO %d", onoff->gpio_pin);
            on_off_plugin_unit::config_t config;
            onoff_endpoint = on_off_plugin_unit::create(node, &config, ENDPOINT_FLAG_NONE, onoff);
            break;
        }
        default:
            ESP_LOGE(TAG, "Unknown on/off device type: %d", (int)onoff->type);
            return;
    }
    
    if (onoff_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create on/off endpoint for GPIO %d", onoff->gpio_pin);
        return;
    }
    onoff->endpoint = endpoint::get_id(onoff_endpoint);

    cluster_t* descriptor = cluster::get(onoff_endpoint, Descriptor::Id);
    descriptor::feature::taglist::add(descriptor);

    ESP_LOGI(TAG, "OnOff device created with endpoint_id %d", onoff->endpoint);

    // Initialize output pin if configured
    if (onoff->output_pin != GPIO_NUM_NC) {
        gpio_reset_pin(onoff->output_pin);
        gpio_set_direction(onoff->output_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(onoff->output_pin, onoff->state ? 1 : 0);
        ESP_LOGI(TAG, "OnOff output pin %d initialised", onoff->output_pin);
    }

    // Create iot_button for the GPIO pin
    button_config_t button_cfg = {
        .long_press_time = 0,  // Not needed for basic on/off
        .short_press_time = 100, // 100ms debounce
    };
    
    button_gpio_config_t gpio_cfg = {
        .gpio_num = onoff->gpio_pin,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };
    
    esp_err_t ret = iot_button_new_gpio_device(&button_cfg, &gpio_cfg, &onoff->button_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create iot_button for GPIO %d: %s", onoff->gpio_pin, esp_err_to_name(ret));
        return;
    }
    
    // Register callback for button press events
    ret = iot_button_register_cb(onoff->button_handle, BUTTON_PRESS_DOWN, nullptr, onoff_button_handler, onoff);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register BUTTON_PRESS_DOWN callback for GPIO %d: %s", onoff->gpio_pin, esp_err_to_name(ret));
        iot_button_delete(onoff->button_handle);
        onoff->button_handle = nullptr;
        return;
    }
}

/**
 * Destroy an on/off device and clean up its resources.
 * 
 * @param onoff Pointer to gpio_onoff structure to destroy.
 */
void destroy_onoff_device(gpio_onoff* onoff)
{
    if (onoff && onoff->button_handle) {
        ESP_LOGI(TAG, "Destroying on/off device on GPIO %d", onoff->gpio_pin);
        iot_button_delete(onoff->button_handle);
        onoff->button_handle = nullptr;
    }
}

/**
 * Using the on/off device list configured via Kconfig, create the devices and add them to the Matter node.
 */
void create_application_onoff_devices(node_t* node)
{    
    // Parse CONFIG_ONOFF_GPIO_LIST for a list of GPIO pins to create on/off devices.
    // This is a space-separated list of GPIO pin definitions.
    // Example: CONFIG_ONOFF_GPIO_LIST="L34:12 O22 S16"
    // L34    - Light on pin 34
    // O22    - Outlet on pin 22
    // L34:12 - Light on pin 34 with output on pin 12

    const char* gpio_list_str = CONFIG_ONOFF_GPIO_LIST;
    if (gpio_list_str && gpio_list_str[0] != '\0') {
        char buf[128];
        strncpy(buf, gpio_list_str, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* token = strtok(buf, " ");
        while (token != nullptr) {
            size_t len = strlen(token);
            if (len < 2) {
                ESP_LOGW(TAG, "Invalid on/off device definition: '%s' - skipping", token);
                token = strtok(nullptr, " ");
                continue;
            }

            // Parse device type
            auto type = onoff_type::LIGHT;

            int idx = 0;
            if (token[idx] == 'L') {
                type = onoff_type::LIGHT;
                ++idx;
            } else if (token[idx] == 'O') {
                type = onoff_type::OUTLET;
                ++idx;
            } else {
                ESP_LOGW(TAG, "Unknown device type in on/off definition: '%s' - skipping", token);
                token = strtok(nullptr, " ");
                continue;
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
                const char* type_str;
                switch (type) {
                    case onoff_type::LIGHT: type_str = "LIGHT"; break;
                    case onoff_type::OUTLET: type_str = "OUTLET"; break;
                    default: type_str = "UNKNOWN"; break;
                }
                
                ESP_LOGI(
                    TAG, "Creating on/off device: type=%s, pin=%d, output_pin=%s",
                    type_str, pin,
                    output_pin != -1 ? std::to_string(output_pin).c_str() : "none"
                );

                if (configured_onoff_devices >= CONFIG_ONOFF_COUNT) {
                    ESP_LOGE(TAG, "OnOff device storage full. Cannot create more devices. Max: %d", CONFIG_ONOFF_COUNT);
                    return;
                }

                // Create the device with the parsed configuration                
                gpio_onoff* onoff = &onoff_storage[configured_onoff_devices++];
                onoff->gpio_pin = (gpio_num_t)pin;
                onoff->output_pin = output_pin != -1 ? (gpio_num_t)output_pin : GPIO_NUM_NC;
                onoff->state = false;
                onoff->type = type;

                create_onoff_device(node, onoff);
            }
            token = strtok(nullptr, " ");
        }
    } else {
        ESP_LOGW(TAG, "No GPIO pins configured for on/off devices. Please set CONFIG_ONOFF_GPIO_LIST.");
    }
}

/**
 * Destroy all application on/off devices and clean up their resources.
 */
void destroy_application_onoff_devices(void)
{
    ESP_LOGI(TAG, "Destroying %d application on/off devices", configured_onoff_devices);
    
    for (int i = 0; i < configured_onoff_devices; i++) {
        destroy_onoff_device(&onoff_storage[i]);
    }
    
    configured_onoff_devices = 0;
}

/**
 * Sync all on/off device states to their Matter attributes.
 * 
 * This should be called after the Matter network is available to ensure
 * the initial device states are properly reflected in the attributes.
 */
void sync_onoff_states(void)
{
    ESP_LOGI(TAG, "Syncing %d on/off device states to Matter attributes", configured_onoff_devices);
    
    for (int i = 0; i < configured_onoff_devices; ++i) {
        gpio_onoff* onoff = &onoff_storage[i];
        if (onoff->button_handle != nullptr) {
            esp_matter_attr_val_t val = esp_matter_bool(onoff->state);
            esp_err_t ret = attribute::update(
                onoff->endpoint,
                OnOff::Id,
                OnOff::Attributes::OnOff::Id, 
                &val
            );
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Synced on/off endpoint %d to state %s", 
                    onoff->endpoint, onoff->state ? "ON" : "OFF");
            } else {
                ESP_LOGW(TAG, "Failed to sync on/off endpoint %d: %s", 
                    onoff->endpoint, esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "OnOff device %d has no button handle, skipping sync", i);
        }
    }
}

/**
 * Set the state of an on/off device by endpoint ID.
 *
 * @param endpoint_id The endpoint ID of the device to control.
 * @param state The desired state (true = on, false = off).
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t set_onoff_state(uint16_t endpoint_id, bool state)
{
    // Find the device by endpoint ID
    gpio_onoff* onoff = nullptr;
    for (int i = 0; i < configured_onoff_devices; i++) {
        if (onoff_storage[i].endpoint == endpoint_id) {
            onoff = &onoff_storage[i];
            break;
        }
    }
    
    if (onoff == nullptr) {
        ESP_LOGW(TAG, "OnOff device with endpoint %d not found", endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Update the internal state
    onoff->state = state;
    
    ESP_LOGI(TAG, "Set on/off endpoint %d to state %s", endpoint_id, state ? "ON" : "OFF");
    
    // Update the Matter attribute
    esp_matter_attr_val_t val = esp_matter_bool(state);
    return attribute::update(
        endpoint_id,
        OnOff::Id,
        OnOff::Attributes::OnOff::Id, 
        &val
    );
}

/**
 * Get the state of an on/off device by endpoint ID.
 *
 * @param endpoint_id The endpoint ID of the device to query.
 * @param state Pointer to store the current state.
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t get_onoff_state(uint16_t endpoint_id, bool* state)
{
    if (state == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find the device by endpoint ID
    gpio_onoff* onoff = nullptr;
    for (int i = 0; i < configured_onoff_devices; i++) {
        if (onoff_storage[i].endpoint == endpoint_id) {
            onoff = &onoff_storage[i];
            break;
        }
    }
    
    if (onoff == nullptr) {
        ESP_LOGW(TAG, "OnOff device with endpoint %d not found", endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    *state = onoff->state;
    return ESP_OK;
}

/**
 * Matter attribute update callback for OnOff devices.
 * This handles updates from other Matter devices (controllers, etc.)
 * 
 * @param endpoint_id The endpoint ID that was updated.
 * @param cluster_id The cluster ID that was updated.
 * @param attribute_id The attribute ID that was updated.
 * @param val The new attribute value.
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t onoff_attribute_update_cb(
    uint16_t endpoint_id, 
    uint32_t cluster_id, 
    uint32_t attribute_id, 
    esp_matter_attr_val_t *val
)
{
    // Only handle OnOff cluster updates
    if (cluster_id != OnOff::Id) {
        return ESP_OK;
    }
    
    // Only handle OnOff attribute updates
    if (attribute_id != OnOff::Attributes::OnOff::Id) {
        return ESP_OK;
    }
    
    // Find the device by endpoint ID
    gpio_onoff* onoff = nullptr;
    for (int i = 0; i < configured_onoff_devices; i++) {
        if (onoff_storage[i].endpoint == endpoint_id) {
            onoff = &onoff_storage[i];
            break;
        }
    }
    
    if (onoff == nullptr) {
        // Not one of our OnOff devices, ignore
        return ESP_OK;
    }
    
    // Extract the new state value
    bool new_state = val->val.b;
    
    // Only update if the state actually changed
    if (onoff->state != new_state) {
        onoff->state = new_state;
        
        ESP_LOGI(TAG, "OnOff endpoint %d updated remotely to state %s", endpoint_id, new_state ? "ON" : "OFF");
        
        // Update output pin if configured
        update_onoff_output_pin(onoff);
        
        // Here you could add additional logic like:
        // - Updating physical outputs (LEDs, relays, etc.)
        // - Triggering other actions based on state change
        // - Sending notifications
    }
    
    return ESP_OK;
}
