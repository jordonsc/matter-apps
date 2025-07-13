#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <enable_esp_insights.h>
#include <app_reset.h>
#include <app/util/attribute-storage.h>

#if CONFIG_APP_BUTTON_ENABLED
#include <app_button.h>
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

static const char *TAG = "app_main";

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

// This callback is called for every attribute update. The callback implementation shall handle the desired attributes 
// and return an appropriate error code. If the attribute is not of your interest, please do not return an error code 
// and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        // Driver update
        btn_handle_t driver_handle = (btn_handle_t)priv_data;
        err = button_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

extern "C" void app_main()
{
    // Initialize the ESP NVS layer
    nvs_flash_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

#if CONFIG_APP_BUTTON_ENABLED
    // Create application buttons
    create_application_buttons(node);
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    // Set OpenThread platform config
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    // Start the Matter stack
    esp_err_t err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter stack: error %d", err));

#if CONFIG_ENABLE_ESP_INSIGHTS_TRACE
    enable_insights(insights_auth_key_start);
#endif

    // Button on-off tags
    SetTagList(1, chip::Span<const Descriptor::Structs::SemanticTagStruct::Type>(gEp1TagList));
    SetTagList(2, chip::Span<const Descriptor::Structs::SemanticTagStruct::Type>(gEp2TagList));

#if CONFIG_ENABLE_CHIP_SHELL
    // Matter console
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::init();
#endif
}
