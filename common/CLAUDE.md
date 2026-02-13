# Common - Shared Code

Shared components used by applications in the repo. The `features/` directory contains the core business logic; the rest are platform/infrastructure support.

## Directory Overview

| Directory | Purpose | Used by omni |
|-----------|---------|:---:|
| `features/` | Feature modules (endpoints, callbacks, GPIO/UART drivers) | Yes |
| `app_reset/` | ESP-IDF component: factory reset via long-press GPIO button | Yes |
| `app_bridge/` | ESP-IDF component: Matter bridge for non-Matter devices | No |
| `utils/` | Small headers: macros, ESP Insights, heap logging | Yes |
| `cmake_common/` | CMake include for sdkconfig defaults loading | Yes |
| `blemesh_platform/` | Matter platform HAL (BLE/NimBLE, Thread, OTA, certs, KVS) | Yes |
| `external_platform/` | ESP32-C2 version-specific platform configs | Conditional |
| `relinker/` | ESP32-C2 linker scripts for memory optimisation | Conditional |

## features/ - Feature Modules

These are **not** ESP-IDF components. They're compiled directly as source files via the `main` component's `CMakeLists.txt`:

```cmake
idf_component_register(SRC_DIRS "." "../../common/features" ...)
```

### Module Pattern

Every feature module follows the same structure:

1. **Header** (`app_*.h`): Defines a state struct, enums, and public function signatures
2. **Source** (`app_*.cpp`): Implements the logic with file-scoped static storage
3. **Config parsing**: A `create_application_*()` function parses a Kconfig string (space-separated specs)
4. **Storage**: Static `*_storage` array, `calloc`'d at startup based on parsed count
5. **Counter tracking**: `configured_*` and `max_*` static variables track allocation

### Modules

| File | Matter Cluster | Hardware | Config Format |
|------|---------------|----------|---------------|
| `app_button` | Generic Switch | GPIO input | `[M\|L][D\|L\|X]<pin>[:<output>]` |
| `app_onoff` | On/Off (Light/Outlet) | GPIO input + optional output | `[L\|O]<pin>[:<output>]` |
| `app_sensor` | OccupancySensing / BooleanState | GPIO input | `[O\|G][I]<pin>[:<output>]` |
| `app_hx711` | PressureMeasurement | GPIO (SCK+DOUT) | `<sck>:<dout>[:<scale>]` |
| `app_a01nyub` | PressureMeasurement | UART | `<uart>:<tx>:<rx>` |
| `app_matter` | - (callbacks) | - | N/A |

### Config String Prefixes

**Buttons** (`BUTTON_GPIO_LIST`):
- `M` = momentary, `L` = latching
- `D` = double-click, `L` = long-press, `X` = both
- Example: `MX9:12` = momentary + double-click + long-press on GPIO 9, output on GPIO 12

**OnOff** (`ONOFF_GPIO_LIST`):
- `L` = Light, `O` = Outlet
- Example: `L34:12` = Light on GPIO 34, output on GPIO 12

**Sensors** (`SENSOR_GPIO_LIST`):
- `O` = Occupancy, `G` = Generic (BooleanState)
- `I` suffix = inverted logic
- Example: `OI34:9` = Occupancy sensor on GPIO 34, inverted, output on GPIO 9

### app_matter.cpp - Callback Hub

This file routes Matter framework callbacks to feature-specific handlers:
- `app_event_cb()` - Device events (WiFi, commissioning). Calls `sync_*_states()` on WiFi connect.
- `app_attribute_update_cb()` - Attribute changes from controllers. Delegates to `onoff_attribute_update_cb()` and `hx711_matter_attribute_update_cb()`.
- `app_identification_cb()` - Identify cluster (currently just logs).

When adding a new feature that needs attribute update callbacks, add the delegation call in `app_attribute_update_cb()`.

### HX711 Details

The HX711 module has extra complexity:
- Separate `hx711.h`/`hx711.cpp` - low-level driver class for the HX711 ADC chip
- Uses `PressureMeasurement` cluster to report weight (no native weight cluster in Matter)
- Creates an additional on/off endpoint per sensor for tare functionality
- Persists tare offset to NVS across reboots
- Uses FreeRTOS task for periodic sampling with ring buffer averaging

### A01NYUB Details

- UART-based (not GPIO), uses ESP-IDF UART driver
- Frame protocol: 4-byte frames with 0xFF header and checksum
- Uses FreeRTOS task for polling
- Two modes: processed (steadier, 250ms) and real-time (faster, 150ms)

## app_reset/ Component

Standalone ESP-IDF component (has its own `CMakeLists.txt`). Registers a GPIO button that triggers `esp_matter::factory_reset()` on long-press. Configured via:
- `CONFIG_RESET_GPIO_PIN` (default: 0)
- `CONFIG_RESET_HOLD_TIME` (default: 3000ms)

## Conventions

- All modules use `static const char* TAG = "..."` for ESP logging
- GPIO pins use `gpio_num_t` type, with `GPIO_NUM_NC` for "not connected"
- Output pins are always optional (default `GPIO_NUM_NC`) and initialised LOW
- `iot_button` library is used for all GPIO inputs (buttons, sensors, onoff) for debouncing
- Matter attribute updates go through `chip::DeviceLayer::SystemLayer().ScheduleLambda()` to ensure thread safety
- Endpoint IDs are assigned dynamically by `esp_matter` at creation time and stored in the feature struct
