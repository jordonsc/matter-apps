#include <esp_log.h>
#include <esp_err.h>
#include <esp_matter.h>

using namespace esp_matter;

void app_event_cb(const ChipDeviceEvent *event, intptr_t arg);

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
esp_err_t app_identification_cb(
    identification::callback_type_t type, 
    uint16_t endpoint_id, 
    uint8_t effect_id,
    uint8_t effect_variant, 
    void *priv_data
);

// This callback is called for every attribute update. The callback implementation shall handle the desired attributes 
// and return an appropriate error code. If the attribute is not of your interest, please do not return an error code 
// and strictly return ESP_OK.
esp_err_t app_attribute_update_cb(
    attribute::callback_type_t type, 
    uint16_t endpoint_id, 
    uint32_t cluster_id,
    uint32_t attribute_id, esp_matter_attr_val_t *val, 
    void *priv_data
);
