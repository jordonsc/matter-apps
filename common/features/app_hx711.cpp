#include "app_hx711.h"
#include <hx711.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <cmath>

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char *TAG = "app_hx711";
static const char *NVS_NAMESPACE = "hx711";

static uint16_t configured_hx711_sensors = 0;
static uint16_t max_hx711_sensors = 0;
static struct hx711_sensor *hx711_sensor_storage = nullptr;
static TaskHandle_t hx711_task_handle = NULL;

/**
 * Count the number of valid HX711 sensor configurations in the GPIO list
 */
static uint16_t count_hx711_sensors_in_list(const char *gpio_list_str)
{
    if (!gpio_list_str || gpio_list_str[0] == '\0')
    {
        return 0;
    }

    char buf[256];
    strncpy(buf, gpio_list_str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    uint16_t count = 0;
    char *token = strtok(buf, " ");

    while (token != nullptr)
    {
        // Parse SCK:DOUT or SCK:DOUT:SCALE format
        char *colon1 = strchr(token, ':');
        if (colon1 != nullptr)
        {
            *colon1 = '\0';
            int sck_pin = atoi(token);

            char *dout_str = colon1 + 1;
            char *colon2 = strchr(dout_str, ':');
            int dout_pin;

            if (colon2 != nullptr)
            {
                *colon2 = '\0';
                dout_pin = atoi(dout_str);
            }
            else
            {
                dout_pin = atoi(dout_str);
            }

            // Validate pin ranges
            if (sck_pin >= 0 && sck_pin < GPIO_NUM_MAX &&
                dout_pin >= 0 && dout_pin < GPIO_NUM_MAX)
            {
                count++;
            }
        }
        token = strtok(nullptr, " ");
    }

    return count;
}

/**
 * Read raw value from HX711 sensor
 */
int32_t hx711_read_raw(hx711_sensor *sensor)
{
    if (!sensor || !sensor->active || !sensor->hx711)
    {
        ESP_LOGW(TAG, "HX711 sensor not active");
        return 0;
    }

    if (!sensor->hx711->is_ready())
    {
        // Use timeout-based wait for better reliability
        if (!sensor->hx711->wait_ready_timeout(100, 1))
        {
            ESP_LOGW(TAG, "HX711 timeout - sensor not ready");
            return sensor->last_reading;
        }
    }

    int32_t value = sensor->hx711->read();

    ESP_LOGD(TAG, "HX711 Reading: %ld", value);

    sensor->last_reading = value;
    return value;
}

/**
 * Save tare offset to NVS
 */
static esp_err_t save_tare_offset_to_nvs(uint16_t endpoint_id, long offset)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // Create unique key based on endpoint ID
    char key[16];
    snprintf(key, sizeof(key), "tare_%d", endpoint_id);

    err = nvs_set_i32(nvs_handle, key, offset);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write tare offset to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Saved tare offset %ld to NVS for endpoint %d", offset, endpoint_id);
    }

    nvs_close(nvs_handle);
    return err;
}

/**
 * Load tare offset from NVS
 */
static esp_err_t load_tare_offset_from_nvs(uint16_t endpoint_id, long *offset)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGI(TAG, "NVS namespace not found, no saved tare offset");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        }
        return err;
    }

    // Create unique key based on endpoint ID
    char key[16];
    snprintf(key, sizeof(key), "tare_%d", endpoint_id);

    int32_t stored_offset;
    err = nvs_get_i32(nvs_handle, key, &stored_offset);
    if (err == ESP_OK)
    {
        *offset = stored_offset;
        ESP_LOGI(TAG, "Loaded tare offset %ld from NVS for endpoint %d", *offset, endpoint_id);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No saved tare offset in NVS for endpoint %d", endpoint_id);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read tare offset from NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

/**
 * Tare (zero) the HX711 sensor
 */
void hx711_tare(hx711_sensor *sensor)
{
    if (!sensor || !sensor->active || !sensor->hx711)
    {
        return;
    }

    ESP_LOGI(TAG, "Taring HX711 sensor on pins SCK:%d, DOUT:%d", sensor->sck_pin, sensor->dout_pin);

    // Use the library's tare function which averages 10 readings
    sensor->hx711->tare(10);

    // Update our offset to match
    sensor->offset = sensor->hx711->get_offset();

    ESP_LOGI(TAG, "HX711 tare complete. Offset: %ld", sensor->offset);

    // Save tare offset to NVS
    save_tare_offset_to_nvs(sensor->endpoint, sensor->offset);
}

/**
 * Set scale factor for HX711 sensor
 */
void hx711_set_scale(hx711_sensor *sensor, float scale)
{
    if (!sensor || !sensor->hx711)
    {
        return;
    }

    sensor->scale = scale;
    sensor->hx711->set_scale(scale);

    ESP_LOGI(TAG, "HX711 scale set to: %.6f", scale);
}

/**
 * Get calibrated weight reading from HX711 sensor
 */
float hx711_get_weight(hx711_sensor *sensor, byte samples)
{
    if (!sensor)
    {
        ESP_LOGE(TAG, "HX711 sensor pointer is null");
        return 0.0f;
    }
    if (!sensor->active)
    {
        ESP_LOGE(TAG, "HX711 sensor is not active");
        return 0.0f;
    }
    if (!sensor->hx711)
    {
        ESP_LOGE(TAG, "HX711 instance is not initialized");
        return 0.0f;
    }

    // Use the library's get_units function which applies offset and scale
    return sensor->hx711->get_units(samples);
}

/**
 * Update Matter attribute with current weight reading
 * Implements sample averaging and change threshold filtering
 *
 * @param sensor Pointer to hx711_sensor structure
 * @param force_update If true, bypasses the change threshold check
 */
static void update_hx711_matter_attribute(hx711_sensor *sensor, bool force_update = false)
{
    if (!sensor || !sensor->active)
    {
        return;
    }

    float weight = hx711_get_weight(sensor, CONFIG_HX711_SAMPLE_AVERAGE_COUNT);

    // Calculate change from last reported value
    float change = std::fabs(weight - sensor->last_reported_value);

    // Check if change exceeds threshold (skip reporting if change is small and we've reported before)
    // unless force_update is true
    if (!force_update)
    {
        if (sensor->last_reported_value != 0.0f && change < CONFIG_HX711_CHANGE_THRESHOLD)
        {
            ESP_LOGD(TAG, "HX711 endpoint %d: change %.2f below threshold %.2f, not reporting",
                     sensor->endpoint, change, CONFIG_HX711_CHANGE_THRESHOLD);
            return;
        }
    }

    // Convert weight to integer, clamping to int16_t range to prevent overflow
    int32_t weight_temp = (int32_t)weight;

    // Clamp to int16_t range (-32768 to 32767)
    if (weight_temp > 32767)
    {
        ESP_LOGW(TAG, "HX711 weight %ld exceeds max, clamping to 32767", weight_temp);
        weight_temp = 32767;
    }
    else if (weight_temp < -32768)
    {
        ESP_LOGW(TAG, "HX711 weight %ld below min, clamping to -32768", weight_temp);
        weight_temp = -32768;
    }

    int16_t weight_int = (int16_t)weight_temp;

    ESP_LOGI(TAG, "HX711 endpoint %d: weight=%.2f, change=%.2f, int_value=%d, samples=%d",
             sensor->endpoint, weight, change, weight_int, CONFIG_HX711_SAMPLE_AVERAGE_COUNT);

    // Update the PressureMeasurement cluster with the averaged weight value
    esp_matter_attr_val_t val = esp_matter_int16(weight_int);
    attribute::update(
        sensor->endpoint,
        PressureMeasurement::Id,
        PressureMeasurement::Attributes::MeasuredValue::Id,
        &val);

    // Update last reported value
    sensor->last_reported_value = weight;
}

/**
 * HX711 reading task
 */
static void hx711_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HX711 reading task started");

    while (1)
    {
        for (int i = 0; i < configured_hx711_sensors; i++)
        {
            if (hx711_sensor_storage && hx711_sensor_storage[i].active)
            {
                update_hx711_matter_attribute(&hx711_sensor_storage[i]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_HX711_READING_INTERVAL_MS));
    }
}

/**
 * Initialize HX711 using the library
 */
static void hx711_init_gpio(hx711_sensor *sensor)
{
    if (!sensor)
    {
        return;
    }

    // Create HX711 instance
    sensor->hx711 = new HX711();

    // Initialize HX711 library with DOUT, SCK pins and default gain
    sensor->hx711->begin(sensor->dout_pin, sensor->sck_pin, CONFIG_HX711_GAIN);

    // Set the scale factor if provided
    if (sensor->scale != 1.0f)
    {
        sensor->hx711->set_scale(sensor->scale);
    }

    ESP_LOGI(TAG, "HX711 initialized via library - SCK: %d, DOUT: %d", sensor->sck_pin, sensor->dout_pin);
}

/**
 * Create pressure sensor endpoint for weight measurements
 */
static bool create_pressure_sensor_endpoint(node_t *node, hx711_sensor *sensor)
{
    // Create pressure sensor endpoint
    pressure_sensor::config_t sensor_config;
    endpoint_t *pressure_endpoint = pressure_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    if (pressure_endpoint == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create pressure sensor endpoint");
        return false;
    }

    sensor->endpoint = endpoint::get_id(pressure_endpoint);
    sensor->active = true;

    // --- Descriptor Cluster ---
    cluster_t *descriptor = cluster::get(pressure_endpoint, Descriptor::Id);
    if (descriptor)
    {
        descriptor::feature::taglist::add(descriptor);
    }

    // --- PressureMeasurement Cluster ---
    cluster_t *pressure_cluster = cluster::get(pressure_endpoint, PressureMeasurement::Id);
    if (pressure_cluster)
    {
        // Set measurement range to full int16_t range
        esp_matter_attr_val_t min_val = esp_matter_int16(-32000);
        attribute::set_val(attribute::get(pressure_cluster, PressureMeasurement::Attributes::MinMeasuredValue::Id), &min_val);

        esp_matter_attr_val_t max_val = esp_matter_int16(32000);
        attribute::set_val(attribute::get(pressure_cluster, PressureMeasurement::Attributes::MaxMeasuredValue::Id), &max_val);

        ESP_LOGI(TAG, "Configured pressure measurement range: -32000 to 32000");
    }

    ESP_LOGI(TAG, "Pressure sensor endpoint created: %d", sensor->endpoint);
    return true;
}

/**
 * Create outlet endpoint for tare control
 */
static bool create_tare_outlet_endpoint(node_t *node, hx711_sensor *sensor)
{
    // Create outlet endpoint for tare functionality
    on_off_plugin_unit::config_t tare_config;
    endpoint_t *tare_endpoint = on_off_plugin_unit::create(node, &tare_config, ENDPOINT_FLAG_NONE, NULL);
    if (tare_endpoint == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create tare outlet endpoint");
        return false;
    }

    sensor->tare_button_endpoint = endpoint::get_id(tare_endpoint);

    // --- Descriptor Cluster ---
    cluster_t *descriptor = cluster::get(tare_endpoint, Descriptor::Id);
    if (descriptor)
    {
        descriptor::feature::taglist::add(descriptor);
    }

    // --- OnOff Cluster ---
    cluster_t *on_off_cluster = cluster::get(tare_endpoint, OnOff::Id);
    if (on_off_cluster)
    {
        // Set initial state to OFF
        esp_matter_attr_val_t off_val = esp_matter_bool(false);
        attribute::set_val(attribute::get(on_off_cluster, OnOff::Attributes::OnOff::Id), &off_val);
    }

    ESP_LOGI(TAG, "Tare outlet endpoint created: %d", sensor->tare_button_endpoint);
    return true;
}

/**
 * Create an HX711 load cell sensor and add it to the Matter node
 */
void create_hx711_sensor(node_t *node, hx711_sensor *sensor)
{
    if (!sensor)
    {
        ESP_LOGE(TAG, "Invalid sensor pointer");
        return;
    }

    ESP_LOGI(TAG, "Creating HX711 sensor on pins SCK: %d, DOUT: %d", sensor->sck_pin, sensor->dout_pin);

    if (configured_hx711_sensors >= max_hx711_sensors)
    {
        ESP_LOGE(TAG, "HX711 sensor storage full. Cannot create more sensors. Max: %d", max_hx711_sensors);
        return;
    }

    // Set device name on the root endpoint (endpoint 0) using NodeLabel
    // This names the entire Matter device in controllers like Home Assistant
    if (configured_hx711_sensors == 0)
    {
        endpoint_t *root_endpoint = endpoint::get(node, 0);
        if (root_endpoint)
        {
            cluster_t *basic_info = cluster::get(root_endpoint, BasicInformation::Id);
            if (basic_info)
            {
                char device_name[] = "HX711 Load Cell";
                esp_matter_attr_val_t name_val = esp_matter_char_str(device_name, strlen(device_name));
                esp_err_t err = attribute::set_val(
                    attribute::get(basic_info, BasicInformation::Attributes::NodeLabel::Id),
                    &name_val);
                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG, "Set device NodeLabel to '%s'", device_name);
                }
                else
                {
                    ESP_LOGW(TAG, "Failed to set NodeLabel: %s", esp_err_to_name(err));
                }
            }
        }
    }

    // Initialize GPIO pins
    hx711_init_gpio(sensor);

    // Create pressure sensor endpoint for weight measurements
    if (!create_pressure_sensor_endpoint(node, sensor))
    {
        return;
    }

    // Create tare outlet endpoint for tare control
    if (!create_tare_outlet_endpoint(node, sensor))
    {
        ESP_LOGW(TAG, "Tare endpoint creation failed, continuing without tare control");
        sensor->tare_button_endpoint = 0;
    }

    // Wait for HX711 to settle before taring
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Try to load saved tare offset from NVS
    long saved_offset = 0;
    esp_err_t err = load_tare_offset_from_nvs(sensor->endpoint, &saved_offset);

    if (err == ESP_OK)
    {
        sensor->offset = saved_offset;
        sensor->hx711->set_offset(saved_offset);
        ESP_LOGI(TAG, "Using saved tare offset from NVS: %ld", saved_offset);
    }
    else
    {
        ESP_LOGI(TAG, "No saved tare offset found, performing initial tare");
        hx711_tare(sensor);
    }

    ESP_LOGI(TAG, "HX711 sensor setup complete");
}

/**
 * Destroy an HX711 sensor and clean up its resources
 */
void destroy_hx711_sensor(hx711_sensor *sensor)
{
    if (sensor && sensor->active)
    {
        ESP_LOGI(TAG, "Destroying HX711 sensor on pins SCK:%d, DOUT:%d", sensor->sck_pin, sensor->dout_pin);
        sensor->active = false;

        // Clean up HX711 instance
        if (sensor->hx711)
        {
            delete sensor->hx711;
            sensor->hx711 = nullptr;
        }
    }
}

/**
 * Using the HX711 sensor list configured via Kconfig, create the sensors and add them to the Matter node
 */
void create_application_hx711_sensors(node_t *node)
{
    // Parse CONFIG_HX711_GPIO_LIST for a list of HX711 sensor configurations.
    // Format: "SCK:DOUT SCK:DOUT:SCALE"
    // Example: CONFIG_HX711_GPIO_LIST="2:4 5:18:2048.0"
    // 2:4        - HX711 with SCK on pin 2, DOUT on pin 4
    // 5:18:2048.0 - HX711 with SCK on pin 5, DOUT on pin 18, scale factor 2048.0

    const char *gpio_list_str = CONFIG_HX711_GPIO_LIST;
    if (!gpio_list_str || gpio_list_str[0] == '\0')
    {
        ESP_LOGI(TAG, "No HX711 sensors configured. Please set CONFIG_HX711_GPIO_LIST.");
        return;
    }

    // Count valid sensors in the list
    max_hx711_sensors = count_hx711_sensors_in_list(gpio_list_str);
    if (max_hx711_sensors == 0)
    {
        ESP_LOGI(TAG, "No valid HX711 sensor configurations found. Skipping sensor creation.");
        return;
    }

    // Allocate memory for the sensors
    hx711_sensor_storage = (hx711_sensor *)calloc(max_hx711_sensors, sizeof(hx711_sensor));
    if (!hx711_sensor_storage)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for %d HX711 sensors", max_hx711_sensors);
        return;
    }

    ESP_LOGI(TAG, "Allocated storage for %d HX711 sensors", max_hx711_sensors);

    // Parse and create sensors
    {
        char buf[256];
        strncpy(buf, gpio_list_str, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char *token = strtok(buf, " ");
        while (token != nullptr)
        {
            // Parse SCK:DOUT or SCK:DOUT:SCALE format
            char *colon1 = strchr(token, ':');
            if (colon1 == nullptr)
            {
                ESP_LOGW(TAG, "Invalid HX711 definition: '%s' - missing colon separator", token);
                token = strtok(nullptr, " ");
                continue;
            }

            *colon1 = '\0';
            int sck_pin = atoi(token);

            char *dout_str = colon1 + 1;
            char *colon2 = strchr(dout_str, ':');
            int dout_pin;
            float scale = 1.0f;

            if (colon2 != nullptr)
            {
                *colon2 = '\0';
                dout_pin = atoi(dout_str);
                scale = atof(colon2 + 1);
            }
            else
            {
                dout_pin = atoi(dout_str);
            }

            if (sck_pin < 0 || sck_pin >= GPIO_NUM_MAX)
            {
                ESP_LOGW(TAG, "SCK GPIO pin %d out of range (0-%d) - skipping", sck_pin, GPIO_NUM_MAX - 1);
            }
            else if (dout_pin < 0 || dout_pin >= GPIO_NUM_MAX)
            {
                ESP_LOGW(TAG, "DOUT GPIO pin %d out of range (0-%d) - skipping", dout_pin, GPIO_NUM_MAX - 1);
            }
            else if (configured_hx711_sensors >= max_hx711_sensors)
            {
                ESP_LOGE(TAG, "HX711 sensor storage full. Cannot create more sensors. Max: %d", max_hx711_sensors);
                return;
            }
            else
            {
                ESP_LOGI(TAG, "Creating HX711 sensor: SCK=%d, DOUT=%d, scale=%.6f", sck_pin, dout_pin, scale);

                hx711_sensor *sensor = &hx711_sensor_storage[configured_hx711_sensors];
                sensor->sck_pin = (gpio_num_t)sck_pin;
                sensor->dout_pin = (gpio_num_t)dout_pin;
                sensor->scale = scale;
                sensor->offset = 0;
                sensor->active = false;
                sensor->hx711 = nullptr;

                create_hx711_sensor(node, sensor);
                configured_hx711_sensors++;
            }

            token = strtok(nullptr, " ");
        }
    }

    // Start the reading task if we created any sensors
    if (configured_hx711_sensors > 0)
    {
        xTaskCreate(hx711_task, "hx711_task", 4096, NULL, 5, &hx711_task_handle);
    }
}

/**
 * Destroy all application HX711 sensors and clean up their resources
 */
void destroy_application_hx711_sensors(void)
{
    ESP_LOGI(TAG, "Destroying %d HX711 sensors", configured_hx711_sensors);

    if (hx711_sensor_storage)
    {
        for (int i = 0; i < configured_hx711_sensors; i++)
        {
            destroy_hx711_sensor(&hx711_sensor_storage[i]);
        }

        // Free the dynamically allocated memory
        free(hx711_sensor_storage);
        hx711_sensor_storage = nullptr;
    }

    // Stop the reading task
    if (hx711_task_handle)
    {
        vTaskDelete(hx711_task_handle);
        hx711_task_handle = NULL;
    }

    configured_hx711_sensors = 0;
    max_hx711_sensors = 0;
}

/**
 * Find HX711 sensor by endpoint ID
 */
static hx711_sensor *find_hx711_sensor_by_endpoint(uint16_t endpoint_id)
{
    if (hx711_sensor_storage)
    {
        for (int i = 0; i < configured_hx711_sensors; i++)
        {
            if (hx711_sensor_storage[i].endpoint == endpoint_id)
            {
                return &hx711_sensor_storage[i];
            }
        }
    }
    return nullptr;
}

/**
 * Tare HX711 sensor by endpoint ID
 */
esp_err_t hx711_tare_by_endpoint(uint16_t endpoint_id)
{
    hx711_sensor *sensor = find_hx711_sensor_by_endpoint(endpoint_id);
    if (!sensor)
    {
        ESP_LOGW(TAG, "HX711 sensor with endpoint %d not found", endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Taring HX711 sensor on endpoint %d", endpoint_id);
    hx711_tare(sensor);
    return ESP_OK;
}

/**
 * Find HX711 sensor by button endpoint ID
 */
static hx711_sensor *find_hx711_sensor_by_button_endpoint(uint16_t button_endpoint_id)
{
    if (hx711_sensor_storage)
    {
        for (int i = 0; i < configured_hx711_sensors; i++)
        {
            if (hx711_sensor_storage[i].tare_button_endpoint == button_endpoint_id)
            {
                return &hx711_sensor_storage[i];
            }
        }
    }
    return nullptr;
}

/**
 * Tare HX711 sensor by button endpoint ID
 */
esp_err_t hx711_tare_by_button_endpoint(uint16_t button_endpoint_id)
{
    hx711_sensor *sensor = find_hx711_sensor_by_button_endpoint(button_endpoint_id);
    if (!sensor)
    {
        ESP_LOGW(TAG, "HX711 sensor with button endpoint %d not found", button_endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Taring HX711 sensor via button endpoint %d (sensor endpoint %d)",
             button_endpoint_id, sensor->endpoint);
    hx711_tare(sensor);

    // Update the measurement to reflect the new zero point (force update)
    update_hx711_matter_attribute(sensor, true);
    return ESP_OK;
}

/**
 * Matter command handler for HX711 sensors
 *
 * Custom command IDs:
 * 0x00 - Tare command (zero the scale)
 */
esp_err_t hx711_matter_command_handler(
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t command_id,
    esp_matter_attr_val_t *command_data)
{
    // Only handle our pressure measurement cluster
    if (cluster_id != PressureMeasurement::Id)
    {
        return ESP_OK;
    }

    hx711_sensor *sensor = find_hx711_sensor_by_endpoint(endpoint_id);
    if (!sensor)
    {
        // Not one of our HX711 sensors, ignore
        return ESP_OK;
    }

    ESP_LOGI(TAG, "HX711 command received - endpoint: %d, cluster: 0x%lx, command: 0x%lx",
             endpoint_id, cluster_id, command_id);

    switch (command_id)
    {
    case 0x00: // Tare command
        ESP_LOGI(TAG, "Executing tare command for HX711 endpoint %d", endpoint_id);
        hx711_tare(sensor);

        // Update the measurement to reflect the new zero point (force update)
        update_hx711_matter_attribute(sensor, true);
        return ESP_OK;

    default:
        ESP_LOGD(TAG, "Unknown command 0x%lx for HX711 endpoint %d", command_id, endpoint_id);
        return ESP_OK;
    }
}

/**
 * Find HX711 sensor by tare button endpoint
 */
static hx711_sensor *find_hx711_sensor_by_tare_endpoint(uint16_t tare_endpoint_id)
{
    if (hx711_sensor_storage)
    {
        for (int i = 0; i < configured_hx711_sensors; i++)
        {
            if (hx711_sensor_storage[i].tare_button_endpoint == tare_endpoint_id)
            {
                return &hx711_sensor_storage[i];
            }
        }
    }
    return nullptr;
}

/**
 * Matter attribute update callback for HX711 sensors
 * Handles on/off for tare functionality
 */
esp_err_t hx711_matter_attribute_update_cb(
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t *val)
{
    ESP_LOGI(TAG, "Attribute update - endpoint: %d, cluster: 0x%lx, attribute: 0x%lx",
             endpoint_id, cluster_id, attribute_id);

    // Handle tare on/off light
    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id)
    {
        hx711_sensor *sensor = find_hx711_sensor_by_tare_endpoint(endpoint_id);
        if (sensor && val->type == ESP_MATTER_VAL_TYPE_BOOLEAN)
        {
            bool on_state = val->val.b;
            ESP_LOGD(TAG, "Tare on/off changed to %s for endpoint %d", on_state ? "ON" : "OFF", endpoint_id);

            if (on_state)
            {
                // Tare the scale when turned on
                ESP_LOGI(TAG, "Executing tare operation for HX711 sensor endpoint %d", sensor->endpoint);
                hx711_tare(sensor);
                // Force update after tare to immediately report the new zero point
                update_hx711_matter_attribute(sensor, true);

                // Turn the switch back off after taring
                ESP_LOGD(TAG, "Tare complete, turning switch back off");
                esp_matter_attr_val_t off_val = esp_matter_bool(false);
                attribute::update(
                    endpoint_id,
                    OnOff::Id,
                    OnOff::Attributes::OnOff::Id,
                    &off_val);
            }
            return ESP_OK;
        }
    }

    return ESP_OK;
}