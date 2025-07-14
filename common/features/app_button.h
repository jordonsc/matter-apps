#pragma once

#include <stdlib.h>
#include <string.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_matter.h>
#include <iot_button.h>
#include <button_gpio.h>
#include <hal/gpio_types.h>
#include <app-common/zap-generated/attributes/Accessors.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <esp_openthread_types.h>
#endif

enum class button_mechanism: uint8_t
{
    MOMENTARY = 0,
    LATCHING  = 1,
};

struct gpio_button
{
    gpio_num_t gpio_pin;
    uint16_t endpoint;
    button_mechanism mechanism = button_mechanism::MOMENTARY;
    bool feature_double_click = false;
    bool feature_long_press = false;
    button_handle_t button_handle = nullptr;
};

using namespace esp_matter;

/**
 * Create a button from a GPIO button structure.
 *
 * @param node Pointer to the Matter node.
 * @param button Pointer to gpio_button structure containing button configuration.
 */
void create_button(node_t* node, gpio_button* button);

/**
 * Destroy a button and clean up its resources.
 *
 * @param button Pointer to gpio_button structure to destroy.
 */
void destroy_button(gpio_button* button);

/**
 * Using the button list configured via Kconfig, create the buttons and add them to the Matter node.
 *
 * @param node Pointer to the Matter node to which buttons will be added.
 */
void create_application_buttons(node_t* node);

/**
 * Destroy all application buttons and clean up their resources.
 */
void destroy_application_buttons(void);
