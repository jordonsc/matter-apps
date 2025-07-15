#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <driver/gpio.h>
#include <iot_button.h>

using namespace esp_matter;

enum class sensor_type: uint8_t
{
    OCCUPANCY = 0,
    GENERIC = 1,
};



struct gpio_sensor
{
    gpio_num_t gpio_pin;
    uint16_t endpoint;
    bool state = false;
    bool inverted = false;
    sensor_type type = sensor_type::OCCUPANCY;
    button_handle_t button_handle = nullptr;
};

/**
 * Create a sensor from a GPIO pin.
 *
 * @param node Pointer to the Matter node.
 * @param sensor Pointer to gpio_sensor structure containing sensor configuration.
 */
void create_sensor(node_t* node, gpio_sensor* sensor);

/**
 * Destroy a sensor and clean up its resources.
 *
 * @param sensor Pointer to gpio_sensor structure to destroy.
 */
void destroy_sensor(gpio_sensor* sensor);

/**
 * Using the sensor list configured via Kconfig, create the sensors and add them to the Matter node.
 *
 * This function parses the GPIO list defined in Kconfig and creates sensors for each defined GPIO pin.
 *
 * @param node Pointer to the Matter node to which sensors will be added.
 */
void create_application_sensors(node_t* node);

/**
 * Destroy all application sensors and clean up their resources.
 */
void destroy_application_sensors(void);

/**
 * Sync all sensor states to their Matter attributes.
 * 
 * This should be called after the Matter network is available to ensure
 * the initial sensor states are properly reflected in the attributes.
 */
void sync_sensor_states(void);
