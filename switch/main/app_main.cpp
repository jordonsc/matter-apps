#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <enable_esp_insights.h>
#include <app_priv.h>
#include <app_reset.h>
#include <app/util/attribute-storage.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

static const char *TAG = "app_main";

static uint16_t configured_buttons = 0;
static button_endpoint button_list[CONFIG_MAX_CONFIGURABLE_BUTTONS];
static struct gpio_button button_storage[CONFIG_MAX_CONFIGURABLE_BUTTONS];

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace chip::app::Clusters;

#if CONFIG_ENABLE_ESP_INSIGHTS_TRACE
extern const char insights_auth_key_start[] asm("_binary_insights_auth_key_txt_start");
extern const char insights_auth_key_end[] asm("_binary_insights_auth_key_txt_end");
#endif

namespace {
// Please refer to https://github.com/CHIP-Specifications/connectedhomeip-spec/blob/master/src/namespaces
constexpr const uint8_t kNamespaceSwitches = 0x43;
// Switches Namespace: 0x43, tag 0 (On)
constexpr const uint8_t kTagSwitchOn = 0;
// Switches Namespace: 0x43, tag 1 (Off)
constexpr const uint8_t kTagSwitchOff = 1;

constexpr const uint8_t kNamespacePosition = 8;
// Common Position Namespace: 8, tag: 0 (Left)
constexpr const uint8_t kTagPositionLeft = 0;
// Common Position Namespace: 8, tag: 1 (Right)
constexpr const uint8_t kTagPositionRight = 1;

const Descriptor::Structs::SemanticTagStruct::Type gEp1TagList[] = {
    {.namespaceID = kNamespaceSwitches, .tag = kTagSwitchOn},
    {.namespaceID = kNamespacePosition, .tag = kTagPositionRight}};
const Descriptor::Structs::SemanticTagStruct::Type gEp2TagList[] = {
    {.namespaceID = kNamespaceSwitches, .tag = kTagSwitchOff},
    {.namespaceID = kNamespacePosition, .tag = kTagPositionLeft}};

}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        /* Driver update */
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

/**
 * Create a button and add it to the Matter node.
 *
 * This function initializes the button driver, creates a new endpoint with a switch cluster,
 * and adds the button to the list of configured buttons.
 *
 * @param[in] button Pointer to `gpio_button` structure.
 * @param[in] node Pointer to the Matter node.
 *
 * @return ESP_OK on success.
 * @return ESP_FAIL if node or endpoint creation fails.
 */
static esp_err_t create_button(struct gpio_button* button, node_t* node)
{
    esp_err_t err = ESP_OK;

    /* Initialize driver */
    app_driver_handle_t button_handle = app_driver_button_init(button);

    /* Create a new endpoint. */
    generic_switch::config_t switch_config;
    switch_config.switch_cluster.feature_flags = 
#if CONFIG_GENERIC_SWITCH_TYPE_LATCHING
    cluster::switch_cluster::feature::latching_switch::get_id();
#endif

#if CONFIG_GENERIC_SWITCH_TYPE_MOMENTARY
    cluster::switch_cluster::feature::momentary_switch::get_id();
#endif

    endpoint_t *endpoint = generic_switch::create(node, &switch_config, ENDPOINT_FLAG_NONE, button_handle);

    cluster_t* descriptor = cluster::get(endpoint,Descriptor::Id);
    descriptor::feature::taglist::add(descriptor);

    /* These node and endpoint handles can be used to create/add other endpoints and clusters. */
    if (!node || !endpoint)
    {
        ESP_LOGE(TAG, "Matter node creation failed");
        err = ESP_FAIL;
        return err;
    }

    // Add the button to the list of configured buttons.
    for (int i = 0; i < configured_buttons; i++) {
        if (button_list[i].button == button) {
            break;
        }
    }

    // Check for maximum physical buttons that can be configured.
    if (configured_buttons <CONFIG_MAX_CONFIGURABLE_BUTTONS) {
        button_list[configured_buttons].button = button;
        button_list[configured_buttons].endpoint = endpoint::get_id(endpoint);
        configured_buttons++;
    } else {
        ESP_LOGI(TAG, "Cannot configure more buttons");
        err = ESP_FAIL;
        return err;
    }

    static uint16_t generic_switch_endpoint_id = 0;
    generic_switch_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Generic Switch created with endpoint_id %d", generic_switch_endpoint_id);

    /* Add additional features to the node */
    cluster_t *cluster = cluster::get(endpoint, Switch::Id);

#if CONFIG_GENERIC_SWITCH_TYPE_MOMENTARY
    cluster::switch_cluster::feature::action_switch::add(cluster);
    cluster::switch_cluster::feature::momentary_switch_multi_press::config_t msm;
    msm.multi_press_max = 5;
    cluster::switch_cluster::feature::momentary_switch_multi_press::add(cluster, &msm);
#endif

    return err;
}

/** 
 * Create button from GPIO pin number.
 *
 * This function creates a gpio_button from an integer pin number and calls create_button.
 * It uses static storage to prevent memory issues with callback references.
 *
 * @param[in] pin_number GPIO pin number as integer
 * @param[in] node Matter node pointer
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_NO_MEM if button storage is full.
 * @return Other errors from create_button.
 */
static esp_err_t create_button(int pin_number, node_t* node)
{
    if (configured_buttons >= CONFIG_MAX_CONFIGURABLE_BUTTONS) {
        ESP_LOGE(TAG, "Button storage full. Cannot create more buttons. Max: %d", CONFIG_MAX_CONFIGURABLE_BUTTONS);
        return ESP_ERR_NO_MEM;
    }
    
    button_storage[configured_buttons].GPIO_PIN_VALUE = (gpio_num_t)pin_number;
    return create_button(&button_storage[configured_buttons], node);
}

int get_endpoint(gpio_button* button) {
    for (int i = 0; i < configured_buttons; i++) {
        if (button_list[i].button == button) {
            return button_list[i].endpoint;
        }
    }
    return -1;
}


extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Call for Boot button */
    err = create_button(NULL, node);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to create generic switch button"));
#endif

    // Parse CONFIG_BUTTON_GPIO_LIST for a list of GPIO pins to create buttons.
    // This is a comma-separated list of GPIO pin numbers.
    // Example: CONFIG_BUTTON_GPIO_LIST="9,10,11"
    const char* gpio_list_str = CONFIG_BUTTON_GPIO_LIST;
    if (gpio_list_str && gpio_list_str[0] != '\0') {
        char buf[128];
        strncpy(buf, gpio_list_str, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* token = strtok(buf, ",");
        while (token != nullptr) {
            char* end;
            long pin = strtol(token, &end, 10);
            
            // Check for conversion errors
            if (end == token) {
                ESP_LOGW(TAG, "Invalid GPIO pin format: '%s' - skipping", token);
            } else if (pin < 0 || pin >= GPIO_NUM_MAX) {
                ESP_LOGW(TAG, "GPIO pin %ld out of range (0-%d) - skipping", pin, GPIO_NUM_MAX - 1);
            } else {
                ESP_LOGI(TAG, "Creating button for GPIO pin: %ld", pin);
                create_button((int)pin, node);
            }
            token = strtok(nullptr, ",");
        }
    }
    
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CONFIG_ENABLE_ESP_INSIGHTS_TRACE
    enable_insights(insights_auth_key_start);
#endif

    SetTagList(1, chip::Span<const Descriptor::Structs::SemanticTagStruct::Type>(gEp1TagList));
    SetTagList(2, chip::Span<const Descriptor::Structs::SemanticTagStruct::Type>(gEp2TagList));

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::init();
#endif

}
