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
| `app_sdn` | WindowCovering | UART (RS485) | Single motor address via Kconfig |
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
- `app_attribute_update_cb()` - Attribute changes from controllers. Delegates to `onoff_attribute_update_cb()`, `hx711_matter_attribute_update_cb()`, and `sdn_attribute_update_cb()`.
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

### SDN Blinds Details

- Controls Somfy Sonesse 40 motors via SDN (Somfy Digital Network) protocol over RS485
- Uses a TTL-to-RS485 adapter with auto direction control (no RTS/DE pin — just VCC, GND, TXD, RXD)
- UART at 4800 baud, 8-O-1
- Matter WindowCovering with Lift + Position Aware Lift features (Roller Shade type)
- Maps Matter Percent100ths (0-10000) to SDN percentage (0-100)

**Wire encoding:**
- Raw frame = logical bytes with human-readable field values
- Bus frame (on wire) = every byte EXCEPT the 2-byte checksum is bitwise inverted (`~byte`)
- Checksum = sum of all inverted (bus) bytes excluding checksum itself, big-endian, appended UN-inverted

**Frame structure (raw, before inversion):**
```
[0]     Message ID
[1]     Length (bits 0-6) | directed flag (bit 7)
[2]     Network byte (0xF9=tool→motor, 0xF0=broadcast, 0x9F=motor→tool)
[3-5]   Source address (byte-reversed from display format)
[6-8]   Destination address (byte-reversed from display format)
[9..n-2] Data payload (variable)
[n-1,n]  Checksum (2 bytes, big-endian)
```

**Key message IDs (raw):** CTRL_MOVETO=0x03, CTRL_STOP=0x02, GET_MOTOR_POS=0x0C, POST_MOTOR_POS=0x0D, GET_MOTOR_LIMITS=0x21, SET_NODE_DISC=0x50, GET_NODE_ADDR=0x40, ACK=0x7F, nACK=0x6F

**Discovery sequence:**
1. SET_NODE_DISCOVERY data=0x00 (broadcast) — enter discovery mode
2. GET_NODE_ADDR (broadcast) — poll for motor addresses
3. SET_NODE_DISCOVERY data=0x01 (to motor) — acknowledge
4. SET_NODE_DISCOVERY data=0x00 (broadcast) — end discovery mode
5. GET_MOTOR_LIMITS → stores down_limit_pulses for percentage calculation
6. GET_MOTOR_POSITION → initial position
7. Falls back to broadcast position query if formal discovery fails

**Data payload byte order:** Multi-byte values in SDN data payloads are little-endian. Checksum is big-endian.

**POST_MOTOR_POS response data:** data[0-1] = LE 16-bit pulse position, data[2] = percentage (0-100, 0=open, 100=closed). The percentage byte is used directly — no pulse-to-percentage conversion needed.

**Position tracking:** FreeRTOS task polls every 60s (idle) or 100ms (during movement), using task notifications for immediate wake on new commands.

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
