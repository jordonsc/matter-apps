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
#ifndef CONFIG_APP_SDN_ENABLED
#define CONFIG_APP_SDN_ENABLED 0
#endif

#ifndef CONFIG_SDN_UART_NUM
#define CONFIG_SDN_UART_NUM 1
#endif

#ifndef CONFIG_SDN_TX_PIN
#define CONFIG_SDN_TX_PIN 16
#endif

#ifndef CONFIG_SDN_RX_PIN
#define CONFIG_SDN_RX_PIN 17
#endif

#ifndef CONFIG_SDN_POLL_INTERVAL_MS
#define CONFIG_SDN_POLL_INTERVAL_MS 2000
#endif

// SDN protocol constants
#define SDN_BAUD_RATE           4800
#define SDN_RESPONSE_TIMEOUT_MS 1000
#define SDN_RETRY_COUNT         3
#define SDN_RETRY_DELAY_MS      200
#define SDN_MAX_FRAME_LEN       32

// SDN message IDs (raw, pre-inversion)
#define SDN_MSG_CTRL_MOVE        0x01
#define SDN_MSG_CTRL_STOP        0x02
#define SDN_MSG_CTRL_MOVETO      0x03
#define SDN_MSG_CTRL_MOVEOF      0x04
#define SDN_MSG_GET_MOTOR_POS    0x0C
#define SDN_MSG_POST_MOTOR_POS   0x0D
#define SDN_MSG_GET_MOTOR_LIMITS 0x21
#define SDN_MSG_POST_MOTOR_LIMITS 0x31
#define SDN_MSG_GET_NODE_ADDR    0x40
#define SDN_MSG_SET_NODE_DISC    0x50
#define SDN_MSG_ACK              0x7F
#define SDN_MSG_NACK             0x6F

// Frame structure offsets (raw frame, before bus inversion)
#define SDN_FRAME_MSG_ID         0
#define SDN_FRAME_LENGTH         1   // bits 0-6 = length, bit 7 = directed flag
#define SDN_FRAME_NETWORK        2
#define SDN_FRAME_SRC_ADDR       3   // 3 bytes, byte-reversed from display format
#define SDN_FRAME_DST_ADDR       6   // 3 bytes, byte-reversed from display format
#define SDN_FRAME_DATA           9   // variable length data starts here
#define SDN_FRAME_HEADER_LEN     9   // header before data

// Network byte values (raw)
#define SDN_NET_TOOL_TO_MOTOR    0xF9  // directed tool->motor
#define SDN_NET_BROADCAST        0xF0  // broadcast
#define SDN_NET_MOTOR_TO_TOOL    0x9F  // motor->tool response

// Source address: 01:00:00 displayed = {0x00, 0x00, 0x01} in raw frame (byte-reversed)
#define SDN_SRC_ADDR_0           0x00
#define SDN_SRC_ADDR_1           0x00
#define SDN_SRC_ADDR_2           0x01

// Broadcast destination
#define SDN_BROADCAST_ADDR_0     0xFF
#define SDN_BROADCAST_ADDR_1     0xFF
#define SDN_BROADCAST_ADDR_2     0xFF

enum class sdn_movement_state : uint8_t
{
    IDLE        = 0,
    MOVING_UP   = 1,
    MOVING_DOWN = 2,
};

enum class sdn_pending_cmd : uint8_t
{
    NONE     = 0,
    MOVE_UP  = 1,
    MOVE_DOWN = 2,
    MOVE_TO  = 3,
    STOP     = 4,
};

struct sdn_blind
{
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint8_t address[3];         // Motor's SDN node address (raw frame byte order)
    bool address_known = false; // Whether we've discovered a motor on the bus
    uint16_t endpoint = 0;      // Matter endpoint ID
    uint8_t current_position;   // 0-100%, 0=open, 100=closed
    uint8_t target_position;    // 0-100%, 0=open, 100=closed
    uint16_t up_limit_pulses;   // Up limit in pulses
    uint16_t down_limit_pulses; // Down limit in pulses
    bool active = false;
    bool fault = false;         // Motor in fault state (position unknown or stalled)
    sdn_movement_state movement = sdn_movement_state::IDLE;
    uint8_t stall_count = 0;    // Consecutive polls with no position change during movement
    sdn_pending_cmd pending_cmd = sdn_pending_cmd::NONE;
    uint8_t pending_cmd_pct;    // target percentage for MOVE_TO
};

/**
 * Create an SDN blind and add it to the Matter node.
 */
void create_sdn_blind(node_t *node, sdn_blind *blind);

/**
 * Destroy an SDN blind and clean up its resources.
 */
void destroy_sdn_blind(sdn_blind *blind);

/**
 * Using the SDN configuration from Kconfig, create the blind and add it to the Matter node.
 */
void create_application_sdn_blinds(node_t *node);

/**
 * Destroy all SDN blinds and clean up their resources.
 */
void destroy_application_sdn_blinds(void);

/**
 * Matter attribute update callback for SDN blinds.
 */
esp_err_t sdn_attribute_update_cb(
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t *val
);
