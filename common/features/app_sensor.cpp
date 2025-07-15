#include "app_sensor.h"
#include <button_gpio.h>

#ifndef CONFIG_SENSOR_COUNT
#define CONFIG_SENSOR_COUNT 0
#endif

#ifndef CONFIG_SENSOR_GPIO_LIST
#define CONFIG_SENSOR_GPIO_LIST ""
#endif

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char *TAG = "app_sensor";

static uint16_t configured_sensors = 0;
static struct gpio_sensor sensor_storage[5];


/**
 * Handler for iot_button events.
 */
static void sensor_button_handler(void* button_handle, void* usr_data)
{
    gpio_sensor* sensor = (gpio_sensor*)usr_data;
    button_event_t event = iot_button_get_event((button_handle_t)button_handle);
    
    bool new_state = false;
    
    // Map button events to sensor state
    switch (event) {
        case BUTTON_PRESS_DOWN:
            new_state = true;
            break;
        case BUTTON_PRESS_UP:
            new_state = false;
            break;
        default:
            // Ignore other button events for sensor purposes
            return;
    }

    if (sensor->inverted) {
        // Invert the state if configured
        new_state = !new_state; 
    }

    // Dispatch update only if the state has change
    if (sensor->state != new_state) {
        sensor->state = new_state;
        
        ESP_LOGI(
            TAG, "Sensor endpoint %d on GPIO %d changed state to %s", 
            sensor->endpoint, sensor->gpio_pin, 
            sensor->state ? "HIGH" : "LOW"
        );

        esp_matter_attr_val_t val = esp_matter_bool(sensor->state);
        
        // Update the appropriate cluster based on sensor type
        if (sensor->type == sensor_type::OCCUPANCY) {
            attribute::update(
                sensor->endpoint,
                OccupancySensing::Id,
                OccupancySensing::Attributes::Occupancy::Id, 
                &val
            );
        } else if (sensor->type == sensor_type::GENERIC) {
            attribute::update(
                sensor->endpoint,
                BooleanState::Id,
                BooleanState::Attributes::StateValue::Id, 
                &val
            );
        }
    }
}

/**
 * Create a sensor from a GPIO pin.
 * 
 * Supports both occupancy sensors and generic sensors using BooleanState cluster.
 * 
 * This implementation uses the iot_button component instead of raw GPIO ISR for better debouncing and power 
 * management.
 *
 * @param node Pointer to the Matter node.
 * @param sensor Pointer to `gpio_sensor` structure containing sensor configuration.
 */
void create_sensor(node_t* node, gpio_sensor* sensor)
{
    endpoint_t* sensor_endpoint = nullptr;
    
    if (sensor->type == sensor_type::OCCUPANCY) {
        ESP_LOGI(TAG, "Creating occupancy sensor on GPIO %d", sensor->gpio_pin);

        // The config here is for backwards compatibility with the old API. 
        // It doesn't cover all new features, so ignore the values.
        occupancy_sensor::config_t config;
        config.occupancy_sensing.occupancy_sensor_type = chip::to_underlying(
            OccupancySensing::OccupancySensorTypeEnum::kPir
        );
        config.occupancy_sensing.occupancy_sensor_type_bitmap = chip::to_underlying(
            OccupancySensing::OccupancySensorTypeBitmap::kPir
        );
        
        // Set feature flags for the occupancy sensor
        config.occupancy_sensing.feature_flags = chip::to_underlying(
            OccupancySensing::Feature::kOther
        );

        // Create the occupancy sensor endpoint
        sensor_endpoint = occupancy_sensor::create(node, &config, ENDPOINT_FLAG_NONE, sensor);
    } else if (sensor->type == sensor_type::GENERIC) {
        ESP_LOGI(TAG, "Creating generic sensor on GPIO %d", sensor->gpio_pin);

        // Create a contact sensor endpoint (which uses BooleanState cluster)
        contact_sensor::config_t config;
        
        // Create the contact sensor endpoint
        sensor_endpoint = contact_sensor::create(node, &config, ENDPOINT_FLAG_NONE, sensor);
    }
    
    if (sensor_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create sensor endpoint for GPIO %d", sensor->gpio_pin);
        return;
    }
    sensor->endpoint = endpoint::get_id(sensor_endpoint);

    cluster_t* descriptor = cluster::get(sensor_endpoint, Descriptor::Id);
    descriptor::feature::taglist::add(descriptor);

    ESP_LOGI(TAG, "Sensor created with endpoint_id %d", sensor->endpoint);


    // Create iot_button for the GPIO pin
    button_config_t button_cfg = {
        .long_press_time = 0,  // Not needed for sensor
        .short_press_time = 0, // Not needed for sensor
    };
    
    button_gpio_config_t gpio_cfg = {
        .gpio_num = sensor->gpio_pin,
        .active_level = 0,
        .enable_power_save = false,
        .disable_pull = false,
    };
    
    esp_err_t ret = iot_button_new_gpio_device(&button_cfg, &gpio_cfg, &sensor->button_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create iot_button for GPIO %d: %s", sensor->gpio_pin, esp_err_to_name(ret));
        return;
    }
    
    // Register callbacks for button press/release events
    ret = iot_button_register_cb(sensor->button_handle, BUTTON_PRESS_DOWN, nullptr, sensor_button_handler, sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register BUTTON_PRESS_DOWN callback for GPIO %d: %s", sensor->gpio_pin, esp_err_to_name(ret));
        iot_button_delete(sensor->button_handle);
        sensor->button_handle = nullptr;
        return;
    }
    
    ret = iot_button_register_cb(sensor->button_handle, BUTTON_PRESS_UP, nullptr, sensor_button_handler, sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register BUTTON_PRESS_UP callback for GPIO %d: %s", sensor->gpio_pin, esp_err_to_name(ret));
        iot_button_delete(sensor->button_handle);
        sensor->button_handle = nullptr;
        return;
    }
}

/**
 * Destroy a sensor and clean up its resources.
 * 
 * @param sensor Pointer to gpio_sensor structure to destroy.
 */
void destroy_sensor(gpio_sensor* sensor)
{
    if (sensor && sensor->button_handle) {
        ESP_LOGI(TAG, "Destroying sensor on GPIO %d", sensor->gpio_pin);
        iot_button_delete(sensor->button_handle);
        sensor->button_handle = nullptr;
    }
}

/**
 * Using the sensor list configured via Kconfig, create the sensors and add them to the Matter node.
 */
void create_application_sensors(node_t* node)
{    
    // Parse CONFIG_SENSOR_GPIO_LIST for a list of GPIO pins to create sensors.
    // This is a space-separated list of GPIO pin definitions.
    // Example: CONFIG_SENSOR_GPIO_LIST="OI34 G12"
    // O34  - Occupancy sensor on pin 34
    // OI34 - Occupancy sensor on pin 34 with logic inverted
    // G12  - Generic sensor (BooleanState cluster) on pin 12
    // GI12 - Generic sensor on pin 12 with logic inverted

    const char* gpio_list_str = CONFIG_SENSOR_GPIO_LIST;
    if (gpio_list_str && gpio_list_str[0] != '\0') {
        char buf[128];
        strncpy(buf, gpio_list_str, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* token = strtok(buf, " ");
        while (token != nullptr) {
            size_t len = strlen(token);
            if (len < 2) {
                ESP_LOGW(TAG, "Invalid sensor definition: '%s' - skipping", token);
                token = strtok(nullptr, " ");
                continue;
            }

            // Parse mechanism and features
            auto type = sensor_type::OCCUPANCY;
            auto inverted = false;

            int idx = 0;
            if (token[idx] == 'O') {
                type = sensor_type::OCCUPANCY;
                ++idx;
            } else if (token[idx] == 'G') {
                type = sensor_type::GENERIC;
                ++idx;
            } else {
                ESP_LOGW(TAG, "Unknown mechanism in sensor definition: '%s' - skipping", token);
                token = strtok(nullptr, " ");
                continue;
            }

            // Parse features
            while (idx < len && (token[idx] == 'I')) {
                if (token[idx] == 'I') {
                    inverted = true;
                }
                ++idx;
            }

            // Parse pin number
            int pin = atoi(token + idx);
            if (pin < 0 || pin >= GPIO_NUM_MAX) {
                ESP_LOGW(TAG, "GPIO pin %d out of range (0-%d) - skipping", pin, GPIO_NUM_MAX - 1);
            } else {
                ESP_LOGI(
                    TAG, "Creating sensor: type=%s, inverted=%d, pin=%d",
                    type == sensor_type::OCCUPANCY ? "OCCUPANCY" : 
                    type == sensor_type::GENERIC ? "GENERIC" : "UNKNOWN",
                    inverted, 
                    pin
                );

                if (configured_sensors >= CONFIG_SENSOR_COUNT) {
                    ESP_LOGE(TAG, "Sensor storage full. Cannot create more sensors. Max: %d", CONFIG_SENSOR_COUNT);
                    return;
                }

                // Create the sensor with the parsed features                
                gpio_sensor* sensor = &sensor_storage[configured_sensors++];
                sensor->gpio_pin = (gpio_num_t)pin;
                sensor->inverted = inverted;
                sensor->state = false;
                sensor->type = type;

                create_sensor(node, sensor);
            }
            token = strtok(nullptr, " ");
        }
    } else {
        ESP_LOGW(TAG, "No GPIO pins configured for buttons. Please set CONFIG_BUTTON_GPIO_LIST.");
    }
}

/**
 * Destroy all application sensors and clean up their resources.
 */
void destroy_application_sensors(void)
{
    ESP_LOGI(TAG, "Destroying %d application sensors", configured_sensors);
    
    for (int i = 0; i < configured_sensors; i++) {
        destroy_sensor(&sensor_storage[i]);
    }
    
    configured_sensors = 0;
}

/**
 * Sync all sensor states to their Matter attributes.
 * 
 * This should be called after the Matter network is available to ensure
 * the initial sensor states are properly reflected in the attributes.
 */
void sync_sensor_states(void)
{
    ESP_LOGI(TAG, "Syncing %d sensor states to Matter attributes", configured_sensors);
    
    for (int i = 0; i < configured_sensors; ++i) {
        gpio_sensor* sensor = &sensor_storage[i];
        if (sensor->button_handle != nullptr) {
            esp_matter_attr_val_t val = esp_matter_bool(sensor->state);
            esp_err_t ret;
            
            // Update the appropriate cluster based on sensor type
            if (sensor->type == sensor_type::OCCUPANCY) {
                ret = attribute::update(
                    sensor->endpoint,
                    OccupancySensing::Id,
                    OccupancySensing::Attributes::Occupancy::Id, 
                    &val
                );
            } else if (sensor->type == sensor_type::GENERIC) {
                ret = attribute::update(
                    sensor->endpoint,
                    BooleanState::Id,
                    BooleanState::Attributes::StateValue::Id, 
                    &val
                );
            } else {
                ESP_LOGW(TAG, "Unknown sensor type for endpoint %d, skipping sync", sensor->endpoint);
                continue;
            }
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Synced sensor endpoint %d to state %s", 
                    sensor->endpoint, sensor->state ? "HIGH" : "LOW");
            } else {
                ESP_LOGW(TAG, "Failed to sync sensor endpoint %d: %s", 
                    sensor->endpoint, esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Sensor %d has no button handle, skipping sync", i);
        }
    }
}
