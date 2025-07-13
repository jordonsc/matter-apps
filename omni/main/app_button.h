#pragma once

#include <esp_err.h>
#include <esp_matter.h>
#include <hal/gpio_types.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

struct gpio_button
{
      gpio_num_t GPIO_PIN_VALUE;
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
 * Create button from GPIO pin number.
 * 
 * This will create an iot_button from a gpio_button.
 *
 * @param[in] pin_number GPIO pin number for the button.
 * @param[in] node Pointer to the Matter node.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_NO_MEM if button storage exceeded.
 * @return ESP_FAIL if node or endpoint creation fails.
 */
esp_err_t create_button(int pin_number, node_t* node);

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
void create_application_buttons(node_t *node);


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
btn_handle_t button_init(gpio_button *button = NULL);

/** 
 * Driver Update
 *
 * This API should be called to update the driver for the attribute being updated.
 * This is usually called from the common `app_attribute_update_cb()`.
 *
 * @param[in] endpoint_id Endpoint ID of the attribute.
 * @param[in] cluster_id Cluster ID of the attribute.
 * @param[in] attribute_id Attribute ID of the attribute.
 * @param[in] val Pointer to `esp_matter_attr_val_t`. Use appropriate elements as per the value type.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t button_attribute_update(
    btn_handle_t driver_handle, 
    uint16_t endpoint_id, 
    uint32_t cluster_id,
    uint32_t attribute_id, 
    esp_matter_attr_val_t *val
);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif
