#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <driver/gpio.h>
#include <iot_button.h>
#include <button_gpio.h>

using namespace esp_matter;

enum class onoff_type: uint8_t
{
    LIGHT = 0,
    OUTLET = 1,
    SWITCH = 2,
};

struct gpio_onoff
{
    gpio_num_t gpio_pin;
    gpio_num_t output_pin = GPIO_NUM_NC;  // Optional output pin (GPIO_NUM_NC if not configured)
    uint16_t endpoint;
    bool state = false;
    onoff_type type = onoff_type::LIGHT;
    button_handle_t button_handle = nullptr;
};

/**
 * Create an on/off device from a GPIO pin.
 *
 * @param node Pointer to the Matter node.
 * @param onoff Pointer to gpio_onoff structure containing device configuration.
 */
void create_onoff_device(node_t* node, gpio_onoff* onoff);

/**
 * Destroy an on/off device and clean up its resources.
 *
 * @param onoff Pointer to gpio_onoff structure to destroy.
 */
void destroy_onoff_device(gpio_onoff* onoff);

/**
 * Using the on/off device list configured via Kconfig, create the devices and add them to the Matter node.
 *
 * This function parses the GPIO list defined in Kconfig and creates on/off devices for each defined GPIO pin.
 *
 * @param node Pointer to the Matter node to which devices will be added.
 */
void create_application_onoff_devices(node_t* node);

/**
 * Destroy all application on/off devices and clean up their resources.
 */
void destroy_application_onoff_devices(void);

/**
 * Sync all on/off device states to their Matter attributes.
 * 
 * This should be called after the Matter network is available to ensure
 * the initial device states are properly reflected in the attributes.
 */
void sync_onoff_states(void);

/**
 * Set the state of an on/off device by endpoint ID.
 *
 * @param endpoint_id The endpoint ID of the device to control.
 * @param state The desired state (true = on, false = off).
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t set_onoff_state(uint16_t endpoint_id, bool state);

/**
 * Get the state of an on/off device by endpoint ID.
 *
 * @param endpoint_id The endpoint ID of the device to query.
 * @param state Pointer to store the current state.
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t get_onoff_state(uint16_t endpoint_id, bool* state);

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
esp_err_t onoff_attribute_update_cb(uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val);
