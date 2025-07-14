#include "app_sensor.h"

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char *TAG = "app_main";

static uint16_t configured_sensors = 0;
static struct gpio_sensor sensor_storage[5];


static void sensor_gpio_handler(void* data)
{
    gpio_sensor* sensor = (gpio_sensor*)data;
    bool new_state = gpio_get_level(sensor->gpio_pin);

    if (sensor->inverted) {
        // Invert the state if configured
        new_state = !new_state; 
    }

    // we only need to notify application layer if occupancy changed
    if (sensor->state != new_state) {
        sensor->state = new_state;
        
        // Cannot use ESP_LOG in the interrupt - put it in the lambda to run in the main loop
        chip::DeviceLayer::SystemLayer().ScheduleLambda([sensor]() {
            ESP_LOGI(
                TAG, "Sensor endpoint %d on GPIO %d changed state to %s", 
                sensor->endpoint, sensor->gpio_pin, 
                sensor->state ? "HIGH" : "LOW"
            );

            esp_matter_attr_val_t val = esp_matter_bool(sensor->state);
            attribute::update(
                sensor->endpoint,
                OccupancySensing::Id,
                OccupancySensing::Attributes::Occupancy::Id, 
                &val
            );
        });
    }
}

/**
 * Create a sensor from a GPIO pin.
 * 
 * Note that this assumes an occupancy sensor, no other sensor types are currently supported.
 *
 * @param node Pointer to the Matter node.
 * @param sensor Pointer to `gpio_sensor` structure containing sensor configuration.
 * @param pull_mode GPIO pull mode (e.g., GPIO_PULLUP_ONLY, GPIO_PULLDOWN_ONLY).
 */
void create_sensor(node_t* node, gpio_sensor* sensor, gpio_pull_mode_t pull_mode)
{
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
    // Again, nothing here is actually used, but we could populate this (and features) with the subtype.
    config.occupancy_sensing.feature_flags = chip::to_underlying(
        OccupancySensing::Feature::kOther
    );

    // Create the sensor endpoint
    endpoint_t* sensor_endpoint = occupancy_sensor::create(node, &config, ENDPOINT_FLAG_NONE, sensor);
    if (sensor_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create sensor endpoint for GPIO %d", sensor->gpio_pin);
        return;
    }
    sensor->endpoint = endpoint::get_id(sensor_endpoint);

    cluster_t* cluster = cluster::get(sensor_endpoint, OccupancySensing::Id);
    cluster_t* descriptor = cluster::get(sensor_endpoint, Descriptor::Id);
    descriptor::feature::taglist::add(descriptor);

    ESP_LOGI(TAG, "Sensor created with endpoint_id %d", sensor->endpoint);

    // Occupancy sensor sub-type(s)
    cluster::occupancy_sensing::feature::radar::add(cluster);

    // Initialise the GPIO pin
    gpio_reset_pin(sensor->gpio_pin);
    gpio_set_intr_type(sensor->gpio_pin, GPIO_INTR_ANYEDGE);
    gpio_set_direction(sensor->gpio_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(sensor->gpio_pin, pull_mode);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(sensor->gpio_pin, sensor_gpio_handler, sensor);
}


/**
 * Using the sensor list configured via Kconfig, create the sensors and add them to the Matter node.
 */
void create_application_sensors(node_t* node)
{    
    // Parse CONFIG_SENSOR_GPIO_LIST for a list of GPIO pins to create sensors.
    // This is a space-separated list of GPIO pin definitions.
    // Example: CONFIG_SENSOR_GPIO_LIST="OI34"
    // O34  - Occupancy sensor on pin 34
    // OI34 - Occupancy sensor on pin 34 with logic inverted

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
                idx++;
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
                idx++;
            }

            // Parse pin number
            int pin = atoi(token + idx);
            if (pin < 0 || pin >= GPIO_NUM_MAX) {
                ESP_LOGW(TAG, "GPIO pin %d out of range (0-%d) - skipping", pin, GPIO_NUM_MAX - 1);
            } else {
                ESP_LOGI(
                    TAG, "Creating sensor: type=%s, inverted=%d, pin=%d",
                    type == sensor_type::OCCUPANCY ? "OCCUPANCY" : "UNKNOWN",
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
                sensor->state = false;
                sensor->inverted = inverted;
                sensor->type = type;
                sensor->subtype = sensor_subtype::GENERAL;  // maybe one day this will be useful; HASS doesn't use it

                create_sensor(node, sensor, GPIO_PULLUP_ONLY);
            }
            token = strtok(nullptr, " ");
        }
    } else {
        ESP_LOGW(TAG, "No GPIO pins configured for buttons. Please set CONFIG_BUTTON_GPIO_LIST.");
    }
}
