#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace esp_matter;

// Configuration defaults
#ifndef CONFIG_A01NYUB_UART_LIST
#define CONFIG_A01NYUB_UART_LIST ""
#endif

#ifndef CONFIG_A01NYUB_BAUD_RATE
#define CONFIG_A01NYUB_BAUD_RATE 9600
#endif

#ifndef CONFIG_A01NYUB_MODE
#define CONFIG_A01NYUB_MODE 1  // 0 = processed mode (steadier), 1 = real-time mode (faster)
#endif

// A01NYUB sensor range limits (in mm)
#define A01NYUB_MIN_DISTANCE_MM 280
#define A01NYUB_MAX_DISTANCE_MM 7500

// UART frame format constants
#define A01NYUB_FRAME_HEADER 0xFF
#define A01NYUB_FRAME_LENGTH 4

struct a01nyub_sensor
{
    uart_port_t uart_port;          // UART port number
    gpio_num_t tx_pin;              // TX pin (connected to sensor RX)
    gpio_num_t rx_pin;              // RX pin (connected to sensor TX)
    uint16_t endpoint;              // Matter endpoint
    uint32_t last_distance_mm;      // Last valid distance reading in mm
    uint32_t last_reported_mm = 0;  // Last distance reported to Matter (for change detection)
    bool active = false;
};

/**
 * Create an A01NYUB ultrasonic distance sensor and add it to the Matter node.
 *
 * Creates a sensor endpoint that reports distance measurements in millimeters.
 *
 * @param node Pointer to the Matter node.
 * @param sensor Pointer to a01nyub_sensor structure containing sensor configuration.
 */
void create_a01nyub_sensor(node_t *node, a01nyub_sensor *sensor);

/**
 * Destroy an A01NYUB sensor and clean up its resources.
 *
 * @param sensor Pointer to a01nyub_sensor structure to destroy.
 */
void destroy_a01nyub_sensor(a01nyub_sensor *sensor);

/**
 * Using the A01NYUB sensor list configured via Kconfig, create the sensors and add them to the Matter node.
 *
 * This function parses the UART configuration list defined in Kconfig and creates A01NYUB sensors
 * for each defined configuration.
 *
 * @param node Pointer to the Matter node to which sensors will be added.
 */
void create_application_a01nyub_sensors(node_t *node);

/**
 * Destroy all application A01NYUB sensors and clean up their resources.
 */
void destroy_application_a01nyub_sensors(void);
