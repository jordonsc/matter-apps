# Omni Application

A single configurable Matter device firmware. All endpoint configuration is done via `idf.py menuconfig` -- no code changes needed to add/remove buttons, sensors, or relays.

## Structure

```
omni/
  CMakeLists.txt          # Project-level CMake (target mapping, compiler flags)
  partitions.csv          # Flash layout (dual OTA slots, NVS, factory data)
  sdkconfig.defaults      # Common build defaults (4MB flash, BLE, OTA, IPv6)
  sdkconfig.defaults.esp32c6   # 8MB flash, USB serial console
  sdkconfig.defaults.esp32c2   # Memory-constrained optimisations
  sdkconfig.defaults.esp32h2   # H2-specific
  sdkconfig.defaults.esp32p4   # P4-specific
  main/
    app_main.cpp          # Entry point - creates Matter node, calls feature init
    Kconfig.projbuild     # All menuconfig options (Omni menu)
    CMakeLists.txt        # Component registration, pulls in common/features/ sources
    idf_component.yml     # ESP-IDF component registry dependencies
```

## Entry Point (app_main.cpp)

Straightforward sequential initialisation:
1. `nvs_flash_init()` - initialise NVS
2. `node::create()` - create Matter root node (endpoint 0) with attribute + identification callbacks
3. Conditionally call `create_application_*()` for each enabled feature
4. Configure OpenThread if enabled
5. `esp_matter::start()` - start the Matter stack
6. Register console commands (diagnostics, wifi, factory reset, OpenThread CLI)

Features are guarded by `#if CONFIG_APP_*_ENABLED` preprocessor checks.

## Configuration (Kconfig)

All under the `Omni` menu in menuconfig. Each feature has an enable toggle and a configuration string.

| Feature | Enable Flag | Config String | Default |
|---------|-------------|---------------|---------|
| Reset | `APP_RESET_ENABLED` (default: y) | `RESET_GPIO_PIN`, `RESET_HOLD_TIME` | Pin 0, 3000ms |
| Switch/Button | `APP_BUTTON_ENABLED` (default: n) | `BUTTON_GPIO_LIST` | `M9` |
| OnOff | `APP_ONOFF_ENABLED` (default: n) | `ONOFF_GPIO_LIST` | `L34` |
| Sensor | `APP_SENSOR_ENABLED` (default: n) | `SENSOR_GPIO_LIST` | `O34` |
| HX711 | `APP_HX711_ENABLED` (default: n) | `HX711_GPIO_LIST`, `HX711_READING_INTERVAL_MS` | `2:4`, 1000ms |
| A01NYUB | `APP_A01NYUB_ENABLED` (default: n) | `A01NYUB_UART_LIST`, `A01NYUB_BAUD_RATE`, `A01NYUB_MODE` | `1:5:15`, 9600, mode 1 |
| SDN Blinds | `APP_SDN_ENABLED` (default: n) | `SDN_UART_NUM`, `SDN_TX_PIN`, `SDN_RX_PIN`, `SDN_POLL_INTERVAL_MS` | UART1, TX:16, RX:17, 2000ms |

See `common/CLAUDE.md` for config string format details.

## Build Targets

The `CMakeLists.txt` maps `IDF_TARGET` to the correct device HAL path automatically. Special handling:
- **ESP32-C2**: Enables `relinker` for memory optimisation, uses `cmake_utilities` component
- **ESP32-P4**: Uses `hollow` device HAL (no dedicated HAL yet)

## Flash Partitions

Dual OTA layout (1.875 MB per slot):
- `ota_0` @ 0x20000 (1.875 MB) - primary firmware
- `ota_1` @ 0x200000 (1.875 MB) - OTA update slot
- `nvs` @ 0x10000 (48 KB) - non-volatile storage
- `fctry` @ 0x3e0000 (24 KB) - factory data
- `esp_secure_cert` @ 0xd000 (8 KB, encrypted)

## Adding a New Feature

1. Create `app_newfeature.h` and `app_newfeature.cpp` in `common/features/`
2. Follow the existing module pattern (state struct, `create_*`/`destroy_*`, `create_application_*` with config parsing)
3. Add Kconfig entries in `omni/main/Kconfig.projbuild` under the Omni menu
4. Add conditional include and `create_application_*()` call in `omni/main/app_main.cpp`
5. If the feature needs attribute update callbacks, add delegation in `common/features/app_matter.cpp`'s `app_attribute_update_cb()`
6. If the feature needs state sync on WiFi connect, add call in `app_event_cb()`
7. Update the CLAUDE.md files to document the new feature

## Version

- `PROJECT_VER`: "1.0"
- `PROJECT_VER_NUMBER`: 1
- Set in `CMakeLists.txt`
