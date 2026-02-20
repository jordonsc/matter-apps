#include "app_matter.h"

#if CONFIG_APP_SENSOR_ENABLED
#include "app_sensor.h"
#endif

#if CONFIG_APP_ONOFF_ENABLED
#include "app_onoff.h"
#endif

#if CONFIG_APP_HX711_ENABLED
#include "app_hx711.h"
#endif

#if CONFIG_APP_SDN_ENABLED
#include "app_sdn.h"
#endif

static const char *TAG = "app_matter";

using namespace esp_matter;
using namespace chip::app::Clusters;

void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
        ESP_LOGI(TAG, "WiFi connectivity changed");
#if CONFIG_APP_SENSOR_ENABLED
        // Sync sensor states to Matter attributes now that WiFi is connected
        sync_sensor_states();
#endif
#if CONFIG_APP_ONOFF_ENABLED
        // Sync onoff device states to Matter attributes now that WiFi is connected
        sync_onoff_states();
#endif
        break;

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
esp_err_t app_identification_cb(
    identification::callback_type_t type, 
    uint16_t endpoint_id, 
    uint8_t effect_id,
    uint8_t effect_variant, 
    void *priv_data
)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall handle the desired attributes 
// and return an appropriate error code. If the attribute is not of your interest, please do not return an error code 
// and strictly return ESP_OK.
esp_err_t app_attribute_update_cb(
    attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id, esp_matter_attr_val_t *val,
    void *priv_data
)
{
#if CONFIG_APP_ONOFF_ENABLED
    // Handle OnOff device attribute updates
    esp_err_t err = onoff_attribute_update_cb(endpoint_id, cluster_id, attribute_id, val);
    if (err != ESP_OK) {
        return err;
    }
#endif

#if CONFIG_APP_HX711_ENABLED
    // Handle HX711 attribute updates (tare on/off light and scale configuration)
    esp_err_t hx711_err = hx711_matter_attribute_update_cb(endpoint_id, cluster_id, attribute_id, val);
    if (hx711_err != ESP_OK) {
        return hx711_err;
    }
#endif

#if CONFIG_APP_SDN_ENABLED
    // Handle SDN blind attribute updates (WindowCovering target position)
    esp_err_t sdn_err = sdn_attribute_update_cb(endpoint_id, cluster_id, attribute_id, val);
    if (sdn_err != ESP_OK) {
        return sdn_err;
    }
#endif

    return ESP_OK;
}
