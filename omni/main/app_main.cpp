#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <nvs_flash.h>

#include <common_macros.h>
#include <enable_esp_insights.h>
#include <app/util/attribute-storage.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <app_openthread.h>
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

// Random bits of the Matter stack (event messages, etc)
#include <app_matter.h>

#if CONFIG_APP_RESET_ENABLED
// Factory reset handling
#include <app_reset.h>
#endif

#if CONFIG_APP_BUTTON_ENABLED
// Button handling
#include <app_button.h>
#endif

#if CONFIG_APP_ONOFF_ENABLED
// OnOff device handling
#include <app_onoff.h>
#endif

#if CONFIG_APP_SENSOR_ENABLED
// Sensor handling
#include <app_sensor.h>
#endif

#if CONFIG_APP_HX711_ENABLED
// HX711 Load Cell handling
#include <app_hx711.h>
#endif


static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace chip::app::Clusters;


extern "C" void app_main()
{
    // Initialize the ESP NVS layer
    nvs_flash_init();

    // Create a Matter node and add the mandatory Root Node device type on endpoint 0
    node::config_t node_config;
    node_t* node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

#if CONFIG_APP_RESET_ENABLED
    // Register reset button commands
    if (register_app_reset() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register app reset commands");
        return;
    }
#endif

#if CONFIG_APP_BUTTON_ENABLED
    // Create button endpoints
    create_application_buttons(node);
#endif

#if CONFIG_APP_ONOFF_ENABLED
    // Create onoff device endpoints
    create_application_onoff_devices(node);
#endif

#if CONFIG_APP_SENSOR_ENABLED
    // Create sensor endpoints
    create_application_sensors(node);
#endif

#if CONFIG_APP_HX711_ENABLED
    // Create HX711 load cell sensor endpoints
    create_application_hx711_sensors(node);
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

#if CONFIG_ENABLE_CHIP_SHELL
    // Matter console
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif
}
