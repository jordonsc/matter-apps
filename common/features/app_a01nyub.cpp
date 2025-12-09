#include "app_a01nyub.h"

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;

static const char *TAG = "app_a01nyub";

static uint16_t configured_a01nyub_sensors = 0;
static uint16_t max_a01nyub_sensors = 0;
static struct a01nyub_sensor *a01nyub_sensor_storage = nullptr;
static TaskHandle_t a01nyub_task_handle = NULL;

// Forward declaration
static void update_a01nyub_matter_attribute(a01nyub_sensor *sensor);

/**
 * Count the number of valid A01NYUB sensor configurations in the UART list
 */
static uint16_t count_a01nyub_sensors_in_list(const char *uart_list_str)
{
    if (!uart_list_str || uart_list_str[0] == '\0')
    {
        return 0;
    }

    char buf[256];
    strncpy(buf, uart_list_str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    uint16_t count = 0;
    char *token = strtok(buf, " ");

    while (token != nullptr)
    {
        // Parse UART:TX:RX format
        char *colon1 = strchr(token, ':');
        if (colon1 != nullptr)
        {
            *colon1 = '\0';
            int uart_num = atoi(token);

            char *tx_str = colon1 + 1;
            char *colon2 = strchr(tx_str, ':');

            if (colon2 != nullptr)
            {
                *colon2 = '\0';
                int tx_pin = atoi(tx_str);
                int rx_pin = atoi(colon2 + 1);

                // Validate UART and pin ranges
                if (uart_num >= 0 && uart_num < UART_NUM_MAX &&
                    tx_pin >= 0 && tx_pin < GPIO_NUM_MAX &&
                    rx_pin >= 0 && rx_pin < GPIO_NUM_MAX)
                {
                    count++;
                }
            }
        }
        token = strtok(nullptr, " ");
    }

    return count;
}

/**
 * Parse A01NYUB data frame and return distance in mm
 * Returns 0 if frame is invalid
 */
static uint32_t parse_a01nyub_frame(const uint8_t *data)
{
    // Verify header
    if (data[0] != A01NYUB_FRAME_HEADER)
    {
        ESP_LOGD(TAG, "Invalid frame header: 0x%02X", data[0]);
        return 0;
    }

    // Verify checksum
    uint8_t checksum = (data[0] + data[1] + data[2]) & 0xFF;
    if (checksum != data[3])
    {
        ESP_LOGW(TAG, "Checksum mismatch: calculated=0x%02X, received=0x%02X", checksum, data[3]);
        return 0;
    }

    // Calculate distance in mm
    uint32_t distance_mm = (data[1] << 8) | data[2];

    // Validate distance range
    if (distance_mm < A01NYUB_MIN_DISTANCE_MM || distance_mm > A01NYUB_MAX_DISTANCE_MM)
    {
        ESP_LOGD(TAG, "Distance %lu mm out of valid range (%d-%d mm)",
                 distance_mm, A01NYUB_MIN_DISTANCE_MM, A01NYUB_MAX_DISTANCE_MM);
        return 0;
    }

    return distance_mm;
}

/**
 * Read distance from A01NYUB sensor via UART
 * Implements frame synchronization by searching for the header byte
 * Blocks until data is available (event-driven, not polling)
 */
static bool read_a01nyub_distance(a01nyub_sensor *sensor)
{
    if (!sensor || !sensor->active)
    {
        return false;
    }

    // Continuously search for frame header (0xFF)
    // uart_read_bytes() blocks until data is available, so this is efficient
    uint8_t byte;

    while (true)
    {
        // Block waiting for data (timeout allows task to be responsive)
        int len = uart_read_bytes(sensor->uart_port, &byte, 1, pdMS_TO_TICKS(1000));
        if (len != 1)
        {
            ESP_LOGD(TAG, "UART%d: No data received (timeout)", sensor->uart_port);
            return false; // Timeout - sensor might be disconnected
        }

        if (byte == A01NYUB_FRAME_HEADER)
        {
            // Found header, read the remaining 3 bytes
            uint8_t data[A01NYUB_FRAME_LENGTH];
            data[0] = byte; // Header

            len = uart_read_bytes(sensor->uart_port, &data[1], 3, pdMS_TO_TICKS(100));
            if (len == 3)
            {
                // Log raw bytes for debugging
                ESP_LOGD(TAG, "UART%d Raw: [0x%02X 0x%02X 0x%02X 0x%02X]",
                         sensor->uart_port, data[0], data[1], data[2], data[3]);

                uint32_t distance_mm = parse_a01nyub_frame(data);
                if (distance_mm > 0)
                {
                    sensor->last_distance_mm = distance_mm;
                    ESP_LOGD(TAG, "A01NYUB UART%d: Distance = %lu mm (%.2f cm)",
                             sensor->uart_port, distance_mm, distance_mm / 10.0f);

                    // Update Matter attribute
                    update_a01nyub_matter_attribute(sensor);

                    return true;
                }
                else
                {
                    ESP_LOGD(TAG, "UART%d: Invalid frame data (bad checksum or out of range)", sensor->uart_port);
                    // Continue searching for next valid frame
                }
            }
            else
            {
                ESP_LOGD(TAG, "UART%d: Incomplete frame after header (%d/3 bytes)", sensor->uart_port, len);
                // Continue searching for next frame
            }
        }
        // If byte != 0xFF, just continue to next byte (searching for header)
    }

    return false;
}

/**
 * Update Matter attribute with current distance reading
 * Only updates if the value has changed to reduce network traffic
 */
static void update_a01nyub_matter_attribute(a01nyub_sensor *sensor)
{
    if (!sensor || !sensor->active || sensor->endpoint == 0)
    {
        return;
    }

    // Only update if value has changed (reduces Matter network traffic)
    if (sensor->last_distance_mm == sensor->last_reported_mm)
    {
        return;
    }

    // Map distance in mm to pressure measurement value (int16_t)
    // A01NYUB range: 280-7500mm
    int16_t distance_value = (int16_t)sensor->last_distance_mm;

    esp_matter_attr_val_t val = esp_matter_int16(distance_value);
    attribute::update(
        sensor->endpoint,
        PressureMeasurement::Id,
        PressureMeasurement::Attributes::MeasuredValue::Id,
        &val);

    sensor->last_reported_mm = sensor->last_distance_mm;

    ESP_LOGI(TAG, "Matter update: endpoint %d = %d mm (%.1f cm)",
             sensor->endpoint, distance_value, distance_value / 10.0f);
}

/**
 * A01NYUB reading task
 * Event-driven: blocks waiting for UART data instead of polling
 */
static void a01nyub_task(void *pvParameters)
{
    ESP_LOGI(TAG, "A01NYUB reading task started (event-driven mode)");

    while (1)
    {
        // Process each sensor sequentially
        // Each sensor's read blocks until data arrives, so no need for delays
        for (int i = 0; i < configured_a01nyub_sensors; i++)
        {
            if (a01nyub_sensor_storage && a01nyub_sensor_storage[i].active)
            {
                read_a01nyub_distance(&a01nyub_sensor_storage[i]);
            }
        }
        // No delay needed - read_a01nyub_distance() blocks until data arrives
    }
}

/**
 * Initialize UART for A01NYUB sensor
 */
static esp_err_t a01nyub_init_uart(a01nyub_sensor *sensor)
{
    if (!sensor)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const uart_config_t uart_config = {
        .baud_rate = CONFIG_A01NYUB_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(sensor->uart_port, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART%d: %s", sensor->uart_port, esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(sensor->uart_port, sensor->tx_pin, sensor->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART%d pins (TX:%d, RX:%d): %s",
                 sensor->uart_port, sensor->tx_pin, sensor->rx_pin, esp_err_to_name(err));
        return err;
    }

    // Configure internal pull-up on RX pin to prevent floating when sensor is disconnected
    gpio_set_pull_mode(sensor->rx_pin, GPIO_PULLUP_ONLY);

    // Configure TX pin to control sensor mode (connects to sensor's RX pin)
    // LOW = real-time mode (150ms), HIGH = processed mode (150-300ms, steadier)
    gpio_set_direction(sensor->tx_pin, GPIO_MODE_OUTPUT);
    if (CONFIG_A01NYUB_MODE == 1)
    {
        gpio_set_level(sensor->tx_pin, 0); // LOW = real-time mode
        ESP_LOGI(TAG, "A01NYUB mode: Real-time (150ms response)");
    }
    else
    {
        gpio_set_level(sensor->tx_pin, 1); // HIGH = processed mode
        ESP_LOGI(TAG, "A01NYUB mode: Processed (steadier, 150-300ms response)");
    }

    // Install UART driver with RX buffer only (we don't transmit to the sensor)
    const int uart_buffer_size = 256;
    err = uart_driver_install(sensor->uart_port, uart_buffer_size, 0, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART%d driver: %s", sensor->uart_port, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "A01NYUB UART%d initialized - TX:%d, RX:%d, Baud:%d (event-driven)",
             sensor->uart_port, sensor->tx_pin, sensor->rx_pin, CONFIG_A01NYUB_BAUD_RATE);

    return ESP_OK;
}

/**
 * Create pressure sensor endpoint for distance measurements
 * Note: Matter doesn't have a standard distance cluster, so we use PressureMeasurement
 * and map distance in mm to the MeasuredValue attribute
 */
static bool create_distance_sensor_endpoint(node_t *node, a01nyub_sensor *sensor)
{
    // Create pressure sensor endpoint (repurposed for distance)
    pressure_sensor::config_t sensor_config;
    endpoint_t *distance_endpoint = pressure_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    if (distance_endpoint == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create distance sensor endpoint");
        return false;
    }

    sensor->endpoint = endpoint::get_id(distance_endpoint);

    // --- Descriptor Cluster ---
    cluster_t *descriptor = cluster::get(distance_endpoint, Descriptor::Id);
    if (descriptor)
    {
        descriptor::feature::taglist::add(descriptor);
    }

    // --- PressureMeasurement Cluster (repurposed for distance in mm) ---
    cluster_t *pressure_cluster = cluster::get(distance_endpoint, PressureMeasurement::Id);
    if (pressure_cluster)
    {
        // Set measurement range to match A01NYUB sensor range (280-7500mm)
        esp_matter_attr_val_t min_val = esp_matter_int16(A01NYUB_MIN_DISTANCE_MM);
        attribute::set_val(attribute::get(pressure_cluster, PressureMeasurement::Attributes::MinMeasuredValue::Id), &min_val);

        esp_matter_attr_val_t max_val = esp_matter_int16(A01NYUB_MAX_DISTANCE_MM);
        attribute::set_val(attribute::get(pressure_cluster, PressureMeasurement::Attributes::MaxMeasuredValue::Id), &max_val);

        ESP_LOGI(TAG, "Configured distance measurement range: %d-%d mm",
                 A01NYUB_MIN_DISTANCE_MM, A01NYUB_MAX_DISTANCE_MM);
    }

    ESP_LOGI(TAG, "Distance sensor endpoint created: %d", sensor->endpoint);
    return true;
}

/**
 * Create an A01NYUB ultrasonic distance sensor and add it to the Matter node
 */
void create_a01nyub_sensor(node_t *node, a01nyub_sensor *sensor)
{
    if (!sensor)
    {
        ESP_LOGE(TAG, "Invalid sensor pointer");
        return;
    }

    ESP_LOGI(TAG, "Creating A01NYUB sensor on UART%d (TX:%d, RX:%d)",
             sensor->uart_port, sensor->tx_pin, sensor->rx_pin);

    if (configured_a01nyub_sensors >= max_a01nyub_sensors)
    {
        ESP_LOGE(TAG, "A01NYUB sensor storage full. Cannot create more sensors. Max: %d", max_a01nyub_sensors);
        return;
    }

    // Initialize UART
    esp_err_t err = a01nyub_init_uart(sensor);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize UART for A01NYUB sensor");
        return;
    }

    // Create Matter endpoint
    if (!create_distance_sensor_endpoint(node, sensor))
    {
        ESP_LOGE(TAG, "Failed to create Matter endpoint for A01NYUB sensor");
        return;
    }

    sensor->active = true;
    sensor->last_distance_mm = 0;

    ESP_LOGI(TAG, "A01NYUB sensor setup complete on UART%d, endpoint: %d",
             sensor->uart_port, sensor->endpoint);
}

/**
 * Destroy an A01NYUB sensor and clean up its resources
 */
void destroy_a01nyub_sensor(a01nyub_sensor *sensor)
{
    if (sensor && sensor->active)
    {
        ESP_LOGI(TAG, "Destroying A01NYUB sensor on UART%d", sensor->uart_port);

        // Uninstall UART driver
        uart_driver_delete(sensor->uart_port);

        sensor->active = false;
    }
}

/**
 * Using the A01NYUB sensor list configured via Kconfig, create the sensors and add them to the Matter node
 */
void create_application_a01nyub_sensors(node_t *node)
{
    // Parse CONFIG_A01NYUB_UART_LIST for a list of A01NYUB sensor configurations.
    // Format: "UART:TX:RX UART:TX:RX"
    // Example: CONFIG_A01NYUB_UART_LIST="1:17:16 2:25:26"
    // 1:17:16 - A01NYUB on UART1 with TX on pin 17, RX on pin 16
    // 2:25:26 - A01NYUB on UART2 with TX on pin 25, RX on pin 26

    const char *uart_list_str = CONFIG_A01NYUB_UART_LIST;
    if (!uart_list_str || uart_list_str[0] == '\0')
    {
        ESP_LOGI(TAG, "No A01NYUB sensors configured. Please set CONFIG_A01NYUB_UART_LIST.");
        return;
    }

    // Count valid sensors in the list
    max_a01nyub_sensors = count_a01nyub_sensors_in_list(uart_list_str);
    if (max_a01nyub_sensors == 0)
    {
        ESP_LOGI(TAG, "No valid A01NYUB sensor configurations found. Skipping sensor creation.");
        return;
    }

    // Allocate memory for the sensors
    a01nyub_sensor_storage = (a01nyub_sensor *)calloc(max_a01nyub_sensors, sizeof(a01nyub_sensor));
    if (!a01nyub_sensor_storage)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for %d A01NYUB sensors", max_a01nyub_sensors);
        return;
    }

    ESP_LOGI(TAG, "Allocated storage for %d A01NYUB sensors", max_a01nyub_sensors);

    // Parse and create sensors
    {
        char buf[256];
        strncpy(buf, uart_list_str, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char *token = strtok(buf, " ");
        while (token != nullptr)
        {
            // Parse UART:TX:RX format
            char *colon1 = strchr(token, ':');
            if (colon1 == nullptr)
            {
                ESP_LOGW(TAG, "Invalid A01NYUB definition: '%s' - missing colon separator", token);
                token = strtok(nullptr, " ");
                continue;
            }

            *colon1 = '\0';
            int uart_num = atoi(token);

            char *tx_str = colon1 + 1;
            char *colon2 = strchr(tx_str, ':');

            if (colon2 == nullptr)
            {
                ESP_LOGW(TAG, "Invalid A01NYUB definition: '%s' - missing second colon", token);
                token = strtok(nullptr, " ");
                continue;
            }

            *colon2 = '\0';
            int tx_pin = atoi(tx_str);
            int rx_pin = atoi(colon2 + 1);

            if (uart_num < 0 || uart_num >= UART_NUM_MAX)
            {
                ESP_LOGW(TAG, "UART port %d out of range (0-%d) - skipping", uart_num, UART_NUM_MAX - 1);
            }
            else if (tx_pin < 0 || tx_pin >= GPIO_NUM_MAX)
            {
                ESP_LOGW(TAG, "TX GPIO pin %d out of range (0-%d) - skipping", tx_pin, GPIO_NUM_MAX - 1);
            }
            else if (rx_pin < 0 || rx_pin >= GPIO_NUM_MAX)
            {
                ESP_LOGW(TAG, "RX GPIO pin %d out of range (0-%d) - skipping", rx_pin, GPIO_NUM_MAX - 1);
            }
            else if (configured_a01nyub_sensors >= max_a01nyub_sensors)
            {
                ESP_LOGE(TAG, "A01NYUB sensor storage full. Cannot create more sensors. Max: %d", max_a01nyub_sensors);
                break;
            }
            else
            {
                ESP_LOGI(TAG, "Creating A01NYUB sensor: UART%d, TX=%d, RX=%d", uart_num, tx_pin, rx_pin);

                a01nyub_sensor *sensor = &a01nyub_sensor_storage[configured_a01nyub_sensors];
                sensor->uart_port = (uart_port_t)uart_num;
                sensor->tx_pin = (gpio_num_t)tx_pin;
                sensor->rx_pin = (gpio_num_t)rx_pin;
                sensor->active = false;
                sensor->endpoint = 0;
                sensor->last_distance_mm = 0;

                create_a01nyub_sensor(node, sensor);
                if (sensor->active)
                {
                    configured_a01nyub_sensors++;
                }
            }

            token = strtok(nullptr, " ");
        }
    }

    // Start the reading task if we created any sensors
    if (configured_a01nyub_sensors > 0)
    {
        xTaskCreate(a01nyub_task, "a01nyub_task", 4096, NULL, 5, &a01nyub_task_handle);
    }
    else
    {
        ESP_LOGW(TAG, "No A01NYUB sensors were successfully created");
    }
}

/**
 * Destroy all application A01NYUB sensors and clean up their resources
 */
void destroy_application_a01nyub_sensors(void)
{
    ESP_LOGI(TAG, "Destroying %d A01NYUB sensors", configured_a01nyub_sensors);

    if (a01nyub_sensor_storage)
    {
        for (int i = 0; i < configured_a01nyub_sensors; i++)
        {
            destroy_a01nyub_sensor(&a01nyub_sensor_storage[i]);
        }

        // Free the dynamically allocated memory
        free(a01nyub_sensor_storage);
        a01nyub_sensor_storage = nullptr;
    }

    // Stop the reading task
    if (a01nyub_task_handle)
    {
        vTaskDelete(a01nyub_task_handle);
        a01nyub_task_handle = NULL;
    }

    configured_a01nyub_sensors = 0;
    max_a01nyub_sensors = 0;
}
