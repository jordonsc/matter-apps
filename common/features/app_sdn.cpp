#include "app_sdn.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <string.h>

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char *TAG = "app_sdn";

static struct sdn_blind *sdn_blind_storage = nullptr;
static uint16_t configured_sdn_blinds = 0;
static TaskHandle_t sdn_task_handle = NULL;

// ---------------------------------------------------------------------------
// SDN protocol helpers
// ---------------------------------------------------------------------------

struct sdn_parsed_frame
{
    uint8_t msg_id;
    uint8_t src_addr[3];
    uint8_t dst_addr[3];
    uint8_t network;
    uint8_t data[SDN_MAX_FRAME_LEN]; // copied from frame, not a pointer
    size_t data_len;
    bool valid;
};

/**
 * Log a hex dump of a buffer at INFO level.
 */
static void sdn_log_hex(const char *prefix, const uint8_t *data, size_t len)
{
    char hex[SDN_MAX_FRAME_LEN * 3 + 1];
    size_t pos = 0;
    for (size_t i = 0; i < len && pos < sizeof(hex) - 3; i++)
    {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X", data[i]);
    }
    ESP_LOGI(TAG, "%s [%d bytes]: %s", prefix, (int)len, hex);
}

/**
 * Build an SDN frame and write it into buf.
 *
 * Wire encoding:
 * - Raw frame: [msg_id][length|flags][network][src_addr x3][dst_addr x3][data...]
 * - Bus frame: every byte except the 2-byte checksum is bitwise inverted (~byte)
 * - Checksum: sum of all inverted (bus) bytes excluding checksum, stored big-endian, appended UN-inverted
 *
 * Returns total frame length including checksum.
 */
static size_t sdn_build_frame(uint8_t *buf, uint8_t msg_id,
                               const uint8_t dst[3], const uint8_t *data, size_t data_len)
{
    size_t total_len = SDN_FRAME_HEADER_LEN + data_len + 2; // +2 for checksum

    bool is_broadcast = (dst[0] == SDN_BROADCAST_ADDR_0 &&
                         dst[1] == SDN_BROADCAST_ADDR_1 &&
                         dst[2] == SDN_BROADCAST_ADDR_2);

    // Build raw frame
    uint8_t raw[SDN_MAX_FRAME_LEN];

    raw[SDN_FRAME_MSG_ID] = msg_id;

    // Length field: bits 0-6 = total frame length, bit 7 = directed flag
    uint8_t length_byte = (uint8_t)(total_len & 0x7F);
    if (!is_broadcast)
    {
        length_byte |= 0x80; // Set directed flag
    }
    raw[SDN_FRAME_LENGTH] = length_byte;

    // Network byte
    raw[SDN_FRAME_NETWORK] = is_broadcast ? SDN_NET_BROADCAST : SDN_NET_TOOL_TO_MOTOR;

    // Source address (01:00:00 -> {0x00, 0x00, 0x01} byte-reversed)
    raw[SDN_FRAME_SRC_ADDR]     = SDN_SRC_ADDR_0;
    raw[SDN_FRAME_SRC_ADDR + 1] = SDN_SRC_ADDR_1;
    raw[SDN_FRAME_SRC_ADDR + 2] = SDN_SRC_ADDR_2;

    // Destination address
    raw[SDN_FRAME_DST_ADDR]     = dst[0];
    raw[SDN_FRAME_DST_ADDR + 1] = dst[1];
    raw[SDN_FRAME_DST_ADDR + 2] = dst[2];

    // Data payload
    if (data_len > 0 && data != nullptr)
    {
        memcpy(&raw[SDN_FRAME_DATA], data, data_len);
    }

    // Invert all bytes for bus frame (except checksum which comes after)
    size_t payload_len = SDN_FRAME_HEADER_LEN + data_len; // bytes before checksum
    for (size_t i = 0; i < payload_len; i++)
    {
        buf[i] = ~raw[i];
    }

    // Calculate checksum: sum of all inverted (bus) bytes
    uint16_t cksum = 0;
    for (size_t i = 0; i < payload_len; i++)
    {
        cksum += buf[i];
    }

    // Append checksum UN-inverted, big-endian
    buf[payload_len]     = (uint8_t)(cksum >> 8);
    buf[payload_len + 1] = (uint8_t)(cksum & 0xFF);

    return total_len;
}

/**
 * Parse an SDN response frame received from the bus.
 *
 * Bus frame bytes are inverted (except last 2 = checksum).
 * This function un-inverts to get the raw frame, validates checksum,
 * and extracts fields.
 *
 * The raw_buf is modified in-place (un-inverted) and data pointer
 * points into it, so the buffer must remain valid while using the result.
 */
static sdn_parsed_frame sdn_parse_frame(uint8_t *raw_buf, size_t len)
{
    sdn_parsed_frame result = {};
    result.valid = false;

    if (len < SDN_FRAME_HEADER_LEN + 2) // minimum: header + checksum
    {
        return result;
    }

    // Save the checksum bytes (un-inverted on wire)
    uint16_t rx_cksum = ((uint16_t)raw_buf[len - 2] << 8) | raw_buf[len - 1];

    // Calculate checksum over the inverted (bus) bytes before un-inverting
    uint16_t calc_cksum = 0;
    for (size_t i = 0; i < len - 2; i++)
    {
        calc_cksum += raw_buf[i];
    }

    if (rx_cksum != calc_cksum)
    {
        ESP_LOGW(TAG, "Checksum mismatch: received 0x%04X, calculated 0x%04X", rx_cksum, calc_cksum);
        return result;
    }

    // Un-invert all bytes except checksum to get raw frame
    for (size_t i = 0; i < len - 2; i++)
    {
        raw_buf[i] = ~raw_buf[i];
    }

    result.msg_id = raw_buf[SDN_FRAME_MSG_ID];
    result.network = raw_buf[SDN_FRAME_NETWORK];

    memcpy(result.src_addr, &raw_buf[SDN_FRAME_SRC_ADDR], 3);
    memcpy(result.dst_addr, &raw_buf[SDN_FRAME_DST_ADDR], 3);

    size_t data_len = len - SDN_FRAME_HEADER_LEN - 2; // subtract header and checksum
    memcpy(result.data, &raw_buf[SDN_FRAME_DATA], data_len);
    result.data_len = data_len;
    result.valid = true;

    return result;
}

/**
 * Send an SDN frame over UART.
 * Flushes the RX buffer before sending to discard stale data/echoes.
 */
static esp_err_t sdn_send_frame(sdn_blind *blind, const uint8_t *data, size_t len)
{
    // Flush any stale data in the RX buffer before sending
    size_t buffered = 0;
    uart_get_buffered_data_len(blind->uart_port, &buffered);
    if (buffered > 0)
    {
        ESP_LOGD(TAG, "Flushing %d stale bytes from RX buffer", (int)buffered);
        uart_flush_input(blind->uart_port);
    }

    sdn_log_hex("TX", data, len);

    int written = uart_write_bytes(blind->uart_port, data, len);
    if (written < 0)
    {
        ESP_LOGE(TAG, "UART write failed");
        return ESP_FAIL;
    }

    // Wait for TX FIFO to fully drain before the transceiver switches back to receive
    esp_err_t err = uart_wait_tx_done(blind->uart_port, pdMS_TO_TICKS(100));
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "UART TX wait failed: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

/**
 * Read a single SDN response frame with timeout.
 *
 * Reads the first byte to get timing, then reads remaining bytes based on
 * the frame length field. Returns number of bytes read, or 0 on timeout/error.
 */
static size_t sdn_receive_frame(sdn_blind *blind, uint8_t *buf, size_t buf_size)
{
    // Read first byte (msg_id on the bus, inverted)
    int n = uart_read_bytes(blind->uart_port, buf, 1, pdMS_TO_TICKS(SDN_RESPONSE_TIMEOUT_MS));
    if (n <= 0)
    {
        ESP_LOGD(TAG, "No RX data (timeout)");
        return 0;
    }

    // Read second byte (length field on the bus, inverted)
    n = uart_read_bytes(blind->uart_port, &buf[1], 1, pdMS_TO_TICKS(50));
    if (n <= 0)
    {
        ESP_LOGW(TAG, "RX: got msg_id byte but no length byte");
        return 0;
    }

    // Un-invert the length byte to get raw length
    uint8_t raw_length = ~buf[1];
    size_t frame_len = raw_length & 0x7F; // bits 0-6 = total frame length

    if (frame_len < SDN_FRAME_HEADER_LEN + 2 || frame_len > buf_size)
    {
        ESP_LOGW(TAG, "RX: invalid frame length %d (raw length byte 0x%02X)", (int)frame_len, raw_length);
        // Drain any remaining bytes
        uint8_t drain[SDN_MAX_FRAME_LEN];
        uart_read_bytes(blind->uart_port, drain, sizeof(drain), pdMS_TO_TICKS(50));
        return 0;
    }

    // Read remaining bytes (we already have 2)
    size_t remaining = frame_len - 2;
    n = uart_read_bytes(blind->uart_port, &buf[2], remaining, pdMS_TO_TICKS(200));
    if (n < (int)remaining)
    {
        ESP_LOGW(TAG, "RX: expected %d more bytes, got %d", (int)remaining, n);
        return 0;
    }

    sdn_log_hex("RX", buf, frame_len);
    return frame_len;
}

/**
 * Send a frame and receive a response, parsing it.
 * Retries up to SDN_RETRY_COUNT times on failure.
 * Returns parsed frame (check .valid).
 */
static sdn_parsed_frame sdn_send_and_receive(sdn_blind *blind, uint8_t msg_id,
                                              const uint8_t dst[3],
                                              const uint8_t *data, size_t data_len)
{
    uint8_t tx_buf[SDN_MAX_FRAME_LEN];
    size_t tx_len = sdn_build_frame(tx_buf, msg_id, dst, data, data_len);

    for (int attempt = 0; attempt < SDN_RETRY_COUNT; attempt++)
    {
        if (attempt > 0)
        {
            ESP_LOGD(TAG, "Retry %d for msg 0x%02X", attempt, msg_id);
            vTaskDelay(pdMS_TO_TICKS(SDN_RETRY_DELAY_MS));
        }

        sdn_send_frame(blind, tx_buf, tx_len);

        uint8_t rx_buf[SDN_MAX_FRAME_LEN];
        size_t rx_len = sdn_receive_frame(blind, rx_buf, sizeof(rx_buf));
        if (rx_len == 0)
        {
            continue;
        }

        sdn_parsed_frame result = sdn_parse_frame(rx_buf, rx_len);
        if (result.valid)
        {
            return result;
        }
    }

    sdn_parsed_frame empty = {};
    return empty;
}

// ---------------------------------------------------------------------------
// SDN commands
// ---------------------------------------------------------------------------

static esp_err_t sdn_move_up(sdn_blind *blind)
{
    ESP_LOGI(TAG, "SDN move UP (endpoint %d)", blind->endpoint);

    // CTRL_MOVETO with data {0x01, 0x00, 0x00, 0x00} = move to up limit
    uint8_t data[] = {0x01, 0x00, 0x00, 0x00};

    blind->movement = sdn_movement_state::MOVING_UP;
    blind->target_position = 0;
    blind->stall_count = 0;

    sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_CTRL_MOVETO, blind->address, data, sizeof(data));
    if (resp.valid && resp.msg_id == SDN_MSG_NACK)
    {
        ESP_LOGW(TAG, "Motor nACK'd move up command");
    }

    return ESP_OK;
}

static esp_err_t sdn_move_down(sdn_blind *blind)
{
    ESP_LOGI(TAG, "SDN move DOWN (endpoint %d)", blind->endpoint);

    // CTRL_MOVETO with data {0x00, 0x00, 0x00, 0x00} = move to down limit
    uint8_t data[] = {0x00, 0x00, 0x00, 0x00};

    blind->movement = sdn_movement_state::MOVING_DOWN;
    blind->target_position = 100;
    blind->stall_count = 0;

    sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_CTRL_MOVETO, blind->address, data, sizeof(data));
    if (resp.valid && resp.msg_id == SDN_MSG_NACK)
    {
        ESP_LOGW(TAG, "Motor nACK'd move down command");
    }

    return ESP_OK;
}

static esp_err_t sdn_stop(sdn_blind *blind)
{
    ESP_LOGI(TAG, "SDN STOP (endpoint %d)", blind->endpoint);

    uint8_t data[] = {0x01};

    blind->movement = sdn_movement_state::IDLE;

    sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_CTRL_STOP, blind->address, data, sizeof(data));
    if (resp.valid && resp.msg_id == SDN_MSG_NACK)
    {
        ESP_LOGW(TAG, "Motor nACK'd stop command");
    }

    return ESP_OK;
}

/**
 * Move to a percentage position (0-100).
 * CTRL_MOVETO data format: {0x04, pct, 0x00, 0x00}
 */
static esp_err_t sdn_move_to_position(sdn_blind *blind, uint8_t position_pct)
{
    ESP_LOGI(TAG, "SDN move to position %d%% (endpoint %d)", position_pct, blind->endpoint);

    uint8_t data[] = {0x04, position_pct, 0x00, 0x00};

    blind->target_position = position_pct;
    blind->stall_count = 0;
    blind->movement = (position_pct > blind->current_position)
                          ? sdn_movement_state::MOVING_DOWN
                          : sdn_movement_state::MOVING_UP;

    sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_CTRL_MOVETO, blind->address, data, sizeof(data));
    if (resp.valid && resp.msg_id == SDN_MSG_NACK)
    {
        ESP_LOGW(TAG, "Motor nACK'd move to position command");
    }

    return ESP_OK;
}

/**
 * Query motor limits to get down_limit_pulses for percentage calculation.
 * Returns true if limits were successfully received.
 */
static bool sdn_query_limits(sdn_blind *blind)
{
    ESP_LOGI(TAG, "Querying motor limits");

    sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_GET_MOTOR_LIMITS,
                                                  blind->address, nullptr, 0);

    if (!resp.valid || resp.msg_id != SDN_MSG_POST_MOTOR_LIMITS)
    {
        ESP_LOGW(TAG, "No valid limits response (msg_id=0x%02X, valid=%d)",
                 resp.msg_id, resp.valid);
        return false;
    }

    if (resp.data_len < 4)
    {
        ESP_LOGW(TAG, "Limits response too short: %d bytes", (int)resp.data_len);
        return false;
    }

    // Up limit from data bytes 0-1, down limit from bytes 2-3 (LE 16-bit)
    blind->up_limit_pulses = (uint16_t)resp.data[0] | ((uint16_t)resp.data[1] << 8);
    blind->down_limit_pulses = (uint16_t)resp.data[2] | ((uint16_t)resp.data[3] << 8);

    ESP_LOGI(TAG, "Motor limits: up=%d, down=%d pulses", blind->up_limit_pulses, blind->down_limit_pulses);
    return true;
}

/**
 * Query the motor's current position.
 * Returns true if a valid position was received and updates blind->current_position.
 */
static bool sdn_query_position(sdn_blind *blind)
{
    sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_GET_MOTOR_POS,
                                                  blind->address, nullptr, 0);

    if (!resp.valid || resp.msg_id != SDN_MSG_POST_MOTOR_POS)
    {
        return false;
    }

    if (resp.data_len < 3)
    {
        ESP_LOGW(TAG, "Position response too short: %d bytes", (int)resp.data_len);
        return false;
    }

    // POST_MOTOR_POS data format:
    //   data[0-1]: LE 16-bit pulse position
    //   data[2]:   percentage (0-100)
    uint16_t pulses = (uint16_t)resp.data[0] | ((uint16_t)resp.data[1] << 8);
    uint8_t pct = resp.data[2];

    if (pulses == 0xFFFF || pct > 100)
    {
        if (!blind->fault)
        {
            ESP_LOGE(TAG, "Motor fault: position unknown (pulses=0x%04X, pct=%d, endpoint %d)",
                     pulses, pct, blind->endpoint);
            blind->fault = true;
            blind->movement = sdn_movement_state::IDLE;
        }
        return false;
    }

    // Valid position received — clear fault if previously set
    if (blind->fault)
    {
        ESP_LOGI(TAG, "Motor fault cleared (endpoint %d)", blind->endpoint);
        blind->fault = false;
    }

    uint8_t old_pos = blind->current_position;
    blind->current_position = pct;

    if (old_pos != pct)
    {
        ESP_LOGI(TAG, "Position update: %d%% -> %d%% (pulses=%u, endpoint %d)",
                 old_pos, pct, pulses, blind->endpoint);
        blind->stall_count = 0;
    }

    // Check if movement has completed
    if (pct == blind->target_position && blind->movement != sdn_movement_state::IDLE)
    {
        ESP_LOGI(TAG, "Movement complete at %d%% (endpoint %d)", pct, blind->endpoint);
        blind->movement = sdn_movement_state::IDLE;
        blind->stall_count = 0;
    }

    // Stall detection: position unchanged during movement
    if (old_pos == pct && blind->movement != sdn_movement_state::IDLE)
    {
        blind->stall_count++;
        if (blind->stall_count >= 20) // ~2s at 100ms polling
        {
            ESP_LOGE(TAG, "Motor stalled: position stuck at %d%% (endpoint %d)", pct, blind->endpoint);
            blind->fault = true;
            blind->movement = sdn_movement_state::IDLE;
            blind->stall_count = 0;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

/**
 * Discover motors on the SDN bus using the Somfy discovery protocol.
 *
 * Sequence:
 * 1. SET_NODE_DISCOVERY data=0x00 (broadcast) — enter discovery mode
 * 2. GET_NODE_ADDR (broadcast) — poll for motor addresses
 * 3. If found: SET_NODE_DISCOVERY data=0x01 (to motor) — acknowledge
 * 4. SET_NODE_DISCOVERY data=0x00 (broadcast) — end discovery mode
 * 5. Query limits and initial position
 */
static bool sdn_discover_motor(sdn_blind *blind)
{
    ESP_LOGI(TAG, "Starting SDN motor discovery...");

    uint8_t broadcast[] = {SDN_BROADCAST_ADDR_0, SDN_BROADCAST_ADDR_1, SDN_BROADCAST_ADDR_2};

    // Step 1: Enter discovery mode
    uint8_t disc_start[] = {0x00};
    uint8_t tx_buf[SDN_MAX_FRAME_LEN];
    size_t tx_len = sdn_build_frame(tx_buf, SDN_MSG_SET_NODE_DISC, broadcast, disc_start, sizeof(disc_start));
    sdn_send_frame(blind, tx_buf, tx_len);

    // Brief delay for motors to wake up
    vTaskDelay(pdMS_TO_TICKS(200));

    // Step 2: Poll for motor addresses
    bool found = false;
    for (int attempt = 0; attempt < 3 && !found; attempt++)
    {
        sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_GET_NODE_ADDR,
                                                      broadcast, nullptr, 0);

        if (resp.valid)
        {
            // The response source address is the motor's address
            memcpy(blind->address, resp.src_addr, 3);
            found = true;

            ESP_LOGI(TAG, "Discovered motor at address %02X:%02X:%02X",
                     blind->address[0], blind->address[1], blind->address[2]);

            // Step 3: Acknowledge discovery to the motor
            uint8_t disc_ack[] = {0x01};
            sdn_parsed_frame ack_resp = sdn_send_and_receive(blind, SDN_MSG_SET_NODE_DISC,
                                                              blind->address, disc_ack, sizeof(disc_ack));
            if (ack_resp.valid && ack_resp.msg_id == SDN_MSG_ACK)
            {
                ESP_LOGI(TAG, "Motor acknowledged discovery");
            }
        }

        if (!found)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Step 4: End discovery mode
    tx_len = sdn_build_frame(tx_buf, SDN_MSG_SET_NODE_DISC, broadcast, disc_start, sizeof(disc_start));
    sdn_send_frame(blind, tx_buf, tx_len);

    // Fallback: try broadcast position query if discovery didn't find anything
    if (!found)
    {
        ESP_LOGI(TAG, "Discovery found no motors, trying broadcast position query...");

        sdn_parsed_frame resp = sdn_send_and_receive(blind, SDN_MSG_GET_MOTOR_POS,
                                                      broadcast, nullptr, 0);

        if (resp.valid && resp.msg_id == SDN_MSG_POST_MOTOR_POS)
        {
            memcpy(blind->address, resp.src_addr, 3);
            found = true;

            ESP_LOGI(TAG, "Found motor via position query at %02X:%02X:%02X",
                     blind->address[0], blind->address[1], blind->address[2]);
        }
    }

    if (found)
    {
        blind->address_known = true;

        // Query limits for percentage calculation
        if (!sdn_query_limits(blind))
        {
            ESP_LOGW(TAG, "Could not get motor limits, position percentage may be inaccurate");
        }

        // Get initial position
        sdn_query_position(blind);
    }
    else
    {
        ESP_LOGW(TAG, "No motors discovered on SDN bus");
    }

    return found;
}

// ---------------------------------------------------------------------------
// Matter attribute updates
// ---------------------------------------------------------------------------

static void sdn_update_matter_position(sdn_blind *blind)
{
    if (!blind || !blind->active || blind->endpoint == 0)
    {
        return;
    }

    // Convert 0-100% to Percent100ths (0-10000)
    uint16_t percent_100ths = (uint16_t)blind->current_position * 100;

    esp_matter_attr_val_t current_val = esp_matter_nullable_uint16(percent_100ths);
    attribute::update(
        blind->endpoint,
        WindowCovering::Id,
        WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id,
        &current_val);

    esp_matter_attr_val_t pct_val = esp_matter_nullable_uint8(blind->current_position);
    attribute::update(
        blind->endpoint,
        WindowCovering::Id,
        WindowCovering::Attributes::CurrentPositionLiftPercentage::Id,
        &pct_val);

    uint8_t op_status = 0;
    switch (blind->movement)
    {
        case sdn_movement_state::MOVING_UP:
            op_status = 0x01;
            break;
        case sdn_movement_state::MOVING_DOWN:
            op_status = 0x02;
            break;
        case sdn_movement_state::IDLE:
        default:
            op_status = 0x00;
            break;
    }

    esp_matter_attr_val_t op_val = esp_matter_uint8(op_status);
    attribute::update(
        blind->endpoint,
        WindowCovering::Id,
        WindowCovering::Attributes::OperationalStatus::Id,
        &op_val);

    // SafetyStatus: bit 3 = PositionFailure
    uint16_t safety = blind->fault ? (1 << 3) : 0;
    esp_matter_attr_val_t safety_val = esp_matter_bitmap16(safety);
    attribute::update(
        blind->endpoint,
        WindowCovering::Id,
        WindowCovering::Attributes::SafetyStatus::Id,
        &safety_val);
}

// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

static void sdn_poll_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SDN poll task started");

    while (1)
    {
        if (!sdn_blind_storage || !sdn_blind_storage->active)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        sdn_blind *blind = sdn_blind_storage;

        // If we haven't discovered a motor yet, keep trying
        if (!blind->address_known)
        {
            sdn_discover_motor(blind);
            if (blind->address_known)
            {
                sdn_update_matter_position(blind);
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
            continue;
        }

        // Process any pending command from the attribute callback
        sdn_pending_cmd cmd = blind->pending_cmd;
        if (cmd != sdn_pending_cmd::NONE)
        {
            blind->pending_cmd = sdn_pending_cmd::NONE;
            switch (cmd)
            {
                case sdn_pending_cmd::MOVE_UP:
                    sdn_move_up(blind);
                    break;
                case sdn_pending_cmd::MOVE_DOWN:
                    sdn_move_down(blind);
                    break;
                case sdn_pending_cmd::MOVE_TO:
                    sdn_move_to_position(blind, blind->pending_cmd_pct);
                    break;
                case sdn_pending_cmd::STOP:
                    sdn_stop(blind);
                    break;
                default:
                    break;
            }
        }

        // Poll motor position
        bool was_faulted = blind->fault;
        bool got_position = sdn_query_position(blind);
        if (got_position || blind->fault != was_faulted)
        {
            sdn_update_matter_position(blind);
        }

        // Poll faster during movement, slow refresh when idle
        // Use task notification so the attribute callback can wake us immediately
        TickType_t delay;
        if (blind->movement != sdn_movement_state::IDLE)
        {
            delay = pdMS_TO_TICKS(100);
        }
        else
        {
            delay = pdMS_TO_TICKS(60000);
        }
        ulTaskNotifyTake(pdTRUE, delay);
    }
}

// ---------------------------------------------------------------------------
// UART initialisation
// ---------------------------------------------------------------------------

static esp_err_t sdn_init_uart(sdn_blind *blind)
{
    const uart_config_t uart_config = {
        .baud_rate = SDN_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(blind->uart_port, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART%d: %s", blind->uart_port, esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(blind->uart_port, blind->tx_pin, blind->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART%d pins (TX:%d, RX:%d): %s",
                 blind->uart_port, blind->tx_pin, blind->rx_pin, esp_err_to_name(err));
        return err;
    }

    const int uart_buffer_size = 256;
    err = uart_driver_install(blind->uart_port, uart_buffer_size, uart_buffer_size, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART%d driver: %s", blind->uart_port, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d initialised: %d baud, 8-O-1, TX:%d RX:%d (auto-direction RS485)",
             blind->uart_port, SDN_BAUD_RATE, blind->tx_pin, blind->rx_pin);

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Matter endpoint creation
// ---------------------------------------------------------------------------

static bool create_window_covering_endpoint(node_t *node, sdn_blind *blind)
{
    // end_product_type 0 = Roller Shade
    window_covering_device::config_t wc_config(0);
    endpoint_t *ep = window_covering_device::create(node, &wc_config, ENDPOINT_FLAG_NONE, NULL);
    if (ep == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create WindowCovering endpoint");
        return false;
    }

    blind->endpoint = endpoint::get_id(ep);

    cluster_t *descriptor = cluster::get(ep, Descriptor::Id);
    if (descriptor)
    {
        descriptor::feature::taglist::add(descriptor);
    }

    cluster_t *wc_cluster = cluster::get(ep, WindowCovering::Id);
    if (wc_cluster)
    {
        cluster::window_covering::feature::lift::config_t lift_config;
        cluster::window_covering::feature::lift::add(wc_cluster, &lift_config);

        cluster::window_covering::feature::position_aware_lift::config_t pa_lift_config;
        pa_lift_config.current_position_lift_percentage = nullable<uint8_t>(0);
        pa_lift_config.target_position_lift_percent_100ths = nullable<uint16_t>(0);
        pa_lift_config.current_position_lift_percent_100ths = nullable<uint16_t>(0);
        cluster::window_covering::feature::position_aware_lift::add(wc_cluster, &pa_lift_config);

        // Add SafetyStatus attribute for fault reporting
        cluster::window_covering::attribute::create_safety_status(wc_cluster, 0);
    }

    ESP_LOGI(TAG, "WindowCovering endpoint created: %d (Roller Shade, lift + position-aware)",
             blind->endpoint);
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void create_sdn_blind(node_t *node, sdn_blind *blind)
{
    if (!blind)
    {
        ESP_LOGE(TAG, "Invalid blind pointer");
        return;
    }

    ESP_LOGI(TAG, "Creating SDN blind on UART%d (TX:%d, RX:%d)",
             blind->uart_port, blind->tx_pin, blind->rx_pin);

    esp_err_t err = sdn_init_uart(blind);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialise UART for SDN blind");
        return;
    }

    if (!create_window_covering_endpoint(node, blind))
    {
        ESP_LOGE(TAG, "Failed to create Matter endpoint for SDN blind");
        return;
    }

    blind->active = true;
    blind->address_known = false;
    blind->current_position = 0;
    blind->target_position = 0;
    blind->up_limit_pulses = 0;
    blind->down_limit_pulses = 0;
    blind->movement = sdn_movement_state::IDLE;

    ESP_LOGI(TAG, "SDN blind setup complete on UART%d, endpoint: %d (motor will be auto-detected)",
             blind->uart_port, blind->endpoint);
}

void destroy_sdn_blind(sdn_blind *blind)
{
    if (blind && blind->active)
    {
        ESP_LOGI(TAG, "Destroying SDN blind on UART%d", blind->uart_port);
        uart_driver_delete(blind->uart_port);
        blind->active = false;
    }
}

void create_application_sdn_blinds(node_t *node)
{
    ESP_LOGI(TAG, "Initialising SDN blinds controller");

    sdn_blind_storage = (sdn_blind *)calloc(1, sizeof(sdn_blind));
    if (!sdn_blind_storage)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for SDN blind");
        return;
    }

    sdn_blind *blind = sdn_blind_storage;
    blind->uart_port = (uart_port_t)CONFIG_SDN_UART_NUM;
    blind->tx_pin = (gpio_num_t)CONFIG_SDN_TX_PIN;
    blind->rx_pin = (gpio_num_t)CONFIG_SDN_RX_PIN;

    create_sdn_blind(node, blind);

    if (blind->active)
    {
        configured_sdn_blinds = 1;
        xTaskCreate(sdn_poll_task, "sdn_task", 4096, NULL, 5, &sdn_task_handle);
    }
    else
    {
        ESP_LOGE(TAG, "SDN blind creation failed");
        free(sdn_blind_storage);
        sdn_blind_storage = nullptr;
    }
}

void destroy_application_sdn_blinds(void)
{
    ESP_LOGI(TAG, "Destroying SDN blinds");

    if (sdn_task_handle)
    {
        vTaskDelete(sdn_task_handle);
        sdn_task_handle = NULL;
    }

    if (sdn_blind_storage)
    {
        destroy_sdn_blind(sdn_blind_storage);
        free(sdn_blind_storage);
        sdn_blind_storage = nullptr;
    }

    configured_sdn_blinds = 0;
}

esp_err_t sdn_attribute_update_cb(
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t *val)
{
    if (cluster_id != WindowCovering::Id)
    {
        return ESP_OK;
    }

    if (configured_sdn_blinds == 0 || !sdn_blind_storage)
    {
        return ESP_OK;
    }

    sdn_blind *blind = nullptr;
    if (sdn_blind_storage->endpoint == endpoint_id)
    {
        blind = sdn_blind_storage;
    }

    if (!blind)
    {
        return ESP_OK;
    }

    if (!blind->address_known)
    {
        ESP_LOGW(TAG, "No motor discovered yet, ignoring command on endpoint %d", endpoint_id);
        return ESP_OK;
    }

    // Handle TargetPositionLiftPercent100ths writes
    if (attribute_id == WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id)
    {
        uint16_t target_100ths = val->val.u16;
        uint8_t target_pct = (uint8_t)(target_100ths / 100);

        ESP_LOGI(TAG, "Target position set to %d%% (%d/10000) on endpoint %d",
                 target_pct, target_100ths, endpoint_id);

        // StopMotion sets target = current while idle/faulted. Send CTRL_STOP to clear errors.
        // During movement, target = current is just the framework stopping motion,
        // but the motor will reach the position via the MOVETO already in flight.
        if (target_pct == blind->current_position &&
            (blind->movement == sdn_movement_state::IDLE || blind->fault))
        {
            blind->pending_cmd = sdn_pending_cmd::STOP;
        }
        else if (target_100ths == 0)
        {
            blind->pending_cmd = sdn_pending_cmd::MOVE_UP;
        }
        else if (target_100ths >= 10000)
        {
            blind->pending_cmd = sdn_pending_cmd::MOVE_DOWN;
        }
        else
        {
            blind->pending_cmd_pct = target_pct;
            blind->pending_cmd = sdn_pending_cmd::MOVE_TO;
        }

        // Wake the SDN task immediately
        if (sdn_task_handle)
        {
            xTaskNotifyGive(sdn_task_handle);
        }
    }

    return ESP_OK;
}
