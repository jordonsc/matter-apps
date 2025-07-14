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

enum class switch_latch_position: uint8_t
{
    OPEN   = 0,
    CLOSED = 1,
};

struct gpio_button
{
    gpio_num_t gpio_pin;
    button_mechanism mechanism = button_mechanism::MOMENTARY;
    bool feature_double_click = false;
    bool feature_long_press = false;
};

struct button_endpoint
{
    gpio_button* button;
    uint16_t endpoint;
};

typedef void* btn_handle_t;

using namespace esp_matter;

/**
 * Create a button from a GPIO button structure.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_NO_MEM if button storage exceeded.
 * @return ESP_FAIL if node or endpoint creation fails.
 */
esp_err_t create_button(struct gpio_button* button, node_t* node);

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
esp_err_t create_button(int pin_number, button_mechanism mechanism, bool dbl, bool lpress, node_t* node);

/**
 * Get the endpoint ID for a given button.
 *
 * This function searches through the configured buttons to find the endpoint ID associated with the button.
 *
 * @param[in] button Pointer to `gpio_button`.
 *
 * @return Endpoint ID if found, -1 if not found.
 */
int get_button_endpoint(gpio_button* button);

/**
 * Using the button list configured via Kconfig, create the buttons and add them to the Matter node.
 */
void create_application_buttons(node_t* node);


/** 
 * Initialize the button driver
 *
 * This initializes the button driver associated with the selected board.
 *
 * @param[in] button Pointer to `gpio_button`.For boot button value is NULL.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
btn_handle_t button_init(gpio_button* button = NULL);
