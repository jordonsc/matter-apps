#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_rom_sys.h>

using namespace esp_matter;

// Configuration defaults
#ifndef CONFIG_HX711_GPIO_LIST
#define CONFIG_HX711_GPIO_LIST ""
#endif

#ifndef CONFIG_HX711_READING_INTERVAL_MS
#define CONFIG_HX711_READING_INTERVAL_MS 250
#endif

#ifndef CONFIG_HX711_SAMPLE_AVERAGE_COUNT
#define CONFIG_HX711_SAMPLE_AVERAGE_COUNT 3
#endif

#ifndef CONFIG_HX711_CHANGE_THRESHOLD
#define CONFIG_HX711_CHANGE_THRESHOLD 5.0f
#endif

#ifndef CONFIG_HX711_GAIN
// Options: 32, 64, 128
#define CONFIG_HX711_GAIN 64
#endif

// Forward declaration
class HX711;

struct hx711_sensor
{
    gpio_num_t sck_pin;                // Serial Clock pin
    gpio_num_t dout_pin;               // Data Output pin
    uint16_t endpoint;                 // Pressure sensor endpoint
    uint16_t tare_button_endpoint = 0; // Tare on/off endpoint
    int32_t offset = 0;                // Tare offset (loaded from NVS)
    float scale = 1.0f;                // Scale factor for calibration
    int32_t last_reading = 0;
    float last_reported_value = 0.0f;                       // Last value reported to Matter (for change detection)
    float sample_buffer[CONFIG_HX711_SAMPLE_AVERAGE_COUNT]; // Ring buffer for averaging
    uint8_t sample_index = 0;                               // Current index in sample buffer
    uint8_t sample_count = 0;                               // Number of samples collected so far
    bool active = false;
    HX711 *hx711 = nullptr; // HX711 library instance
};

/**
 * Create an HX711 load cell sensor and add it to the Matter node.
 *
 * Creates a pressure sensor endpoint that uses the PressureMeasurement cluster
 * to report weight values. The weight is mapped to pressure units where each
 * unit represents 0.01 of the weight measurement unit (e.g., grams).
 *
 * @param node Pointer to the Matter node.
 * @param sensor Pointer to hx711_sensor structure containing sensor configuration.
 */
void create_hx711_sensor(node_t *node, hx711_sensor *sensor);

/**
 * Destroy an HX711 sensor and clean up its resources.
 *
 * @param sensor Pointer to hx711_sensor structure to destroy.
 */
void destroy_hx711_sensor(hx711_sensor *sensor);

/**
 * Using the HX711 sensor list configured via Kconfig, create the sensors and add them to the Matter node.
 *
 * This function parses the GPIO list defined in Kconfig and creates HX711 sensors for each defined configuration.
 * Each sensor creates a pressure sensor endpoint that reports weight measurements through the PressureMeasurement cluster.
 *
 * @param node Pointer to the Matter node to which sensors will be added.
 */
void create_application_hx711_sensors(node_t *node);

/**
 * Destroy all application HX711 sensors and clean up their resources.
 */
void destroy_application_hx711_sensors(void);

/**
 * Read raw value from HX711 sensor
 *
 * @param sensor Pointer to hx711_sensor structure
 * @return Raw ADC value from HX711
 */
int32_t hx711_read_raw(hx711_sensor *sensor);

/**
 * Tare (zero) the HX711 sensor
 *
 * @param sensor Pointer to hx711_sensor structure
 */
void hx711_tare(hx711_sensor *sensor);

/**
 * Set scale factor for HX711 sensor
 *
 * @param sensor Pointer to hx711_sensor structure
 * @param scale Scale factor for weight conversion
 */
void hx711_set_scale(hx711_sensor *sensor, float scale);

/**
 * Get calibrated weight reading from HX711 sensor
 *
 * @param sensor Pointer to hx711_sensor structure
 * @return Weight in the units defined by scale factor
 */
float hx711_get_weight(hx711_sensor *sensor);

/**
 * Matter attribute update callback for HX711 sensors.
 * Handles tare commands and other cluster operations.
 *
 * @param endpoint_id The endpoint ID that received the command
 * @param cluster_id The cluster ID of the command
 * @param command_id The command ID that was received
 * @param command_data Pointer to command data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hx711_matter_command_handler(
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t command_id,
    esp_matter_attr_val_t *command_data);

/**
 * Tare HX711 sensor by endpoint ID
 *
 * @param endpoint_id The endpoint ID of the HX711 sensor to tare
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hx711_tare_by_endpoint(uint16_t endpoint_id);

/**
 * Tare HX711 sensor by button endpoint ID
 *
 * @param button_endpoint_id The endpoint ID of the tare button that was pressed
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hx711_tare_by_button_endpoint(uint16_t button_endpoint_id);

/**
 * Matter attribute update callback for HX711 sensors.
 * Handles on/off cluster for tare functionality.
 *
 * Tare offset is saved to NVS and persists across reboots.
 * Scale is configured at compile-time via SCK:DOUT:SCALE format.
 *
 * @param endpoint_id The endpoint ID that received the update
 * @param cluster_id The cluster ID of the update
 * @param attribute_id The attribute ID that was updated
 * @param val Pointer to the new attribute value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hx711_matter_attribute_update_cb(
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t *val);