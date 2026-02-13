# Matter Apps

ESP32 Matter IoT device firmware. Single monorepo with one application (`omni`) and shared code (`common/`).

## Maintaining These Files

When making changes to this codebase, update the relevant CLAUDE.md files to reflect any architectural changes, new features, renamed files, changed conventions, or new patterns. CLAUDE.md files exist at:
- `/CLAUDE.md` (this file) - project overview, build system, repo structure
- `/common/CLAUDE.md` - shared feature modules and conventions
- `/omni/CLAUDE.md` - omni app configuration and structure

## Project Overview

This is a configuration-driven Matter device framework for ESP32. Users define endpoints (buttons, sensors, relays, etc.) entirely via `menuconfig` without writing code. A single firmware binary (`omni`) supports any combination of GPIO-based and UART-based peripherals.

**SDK stack:** ESP-IDF v5.4.1 + ESP-Matter (connectedhomeip)

**Supported targets:** ESP32, ESP32-C2, ESP32-C3, ESP32-C5, ESP32-C6, ESP32-H2, ESP32-S3, ESP32-P4

## Repository Structure

```
matter-apps/
  init                    # Source this to set up build environment (. ./init)
  sdk-location            # Local SDK paths (git-ignored, copy from sdk-location.default)
  certification_declaration/  # Matter certification DER file
  common/                 # Shared code - see common/CLAUDE.md
    features/             # Feature modules (button, onoff, sensor, hx711, a01nyub, matter)
    app_reset/            # Factory reset component
    app_bridge/           # Matter bridge component (not currently used by omni)
    utils/                # Macros, ESP Insights, heap logging
    cmake_common/         # Shared CMake includes for sdkconfig loading
    blemesh_platform/     # Matter platform HAL (BLE, Thread, OTA, certs)
    external_platform/    # ESP32-C2 version-specific configs
    relinker/             # ESP32-C2 memory optimisation linker scripts
  omni/                   # Main application - see omni/CLAUDE.md
    main/                 # App entry point, Kconfig, component config
    partitions.csv        # Flash partition layout (OTA-ready, dual slot)
    sdkconfig.defaults*   # Per-target build defaults
```

## Build System

**Toolchain:** CMake + ESP-IDF build system (`idf.py`)

### Environment Setup

```bash
# First time: copy and edit SDK paths
cp sdk-location.default sdk-location
# Edit sdk-location with your local esp-idf and esp-matter paths

# Activate environment (must source, not execute)
. ./init
```

### Shell Aliases (available after sourcing init)

| Alias | Command | Purpose |
|-------|---------|---------|
| `idf` | `idf.py` | ESP-IDF CLI |
| `menu` | `idf.py menuconfig` | Open configuration menu |
| `build` | `idf.py build` | Build firmware |
| `flash` | `idf.py -p /dev/ttyACM0 flash monitor` | Flash and monitor |
| `bfm` | `idf.py -p /dev/ttyACM0 build flash monitor` | Build, flash, monitor |
| `target` | `idf.py set-target` | Set chip target |
| `nuke` | `esptool.py -p /dev/ttyACM0 erase_flash` | Full flash erase |

### Build Commands

```bash
cd omni
idf.py set-target esp32c6    # Set target chip
idf.py menuconfig            # Configure features and GPIO pins
idf.py build                 # Build
idf.py -p /dev/ttyACM0 flash monitor  # Flash and monitor
```

### Key Environment Variables

- `ESP_MATTER_PATH` - Path to esp-matter SDK (set by `init`)
- `IDF_PATH` - Path to ESP-IDF (set by `init` via esp-idf's export.sh)
- `ESP_MATTER_DEVICE_PATH` - Auto-set in CMakeLists.txt per target

## Architecture

```
Hardware (GPIO/UART)
    |
Feature Modules (common/features/)  -- each creates Matter endpoints
    |
Matter Node (esp_matter)
    |
BLE/WiFi/Thread transport
    |
Matter Controller (Apple Home, Home Assistant, etc.)
```

All features are conditionally compiled via `CONFIG_APP_*_ENABLED` flags from Kconfig. The entry point (`app_main.cpp`) just creates a Matter node and calls `create_application_*()` for each enabled feature.

## Conventions

- **C++17** throughout (set in CMakeLists.txt)
- **ESP-IDF logging** via `ESP_LOGI`, `ESP_LOGE`, `ESP_LOGW`, `ESP_LOGD` with a `TAG` per file
- **Configuration strings** are parsed at startup from Kconfig - space-separated specs with a type prefix letter and optional `:output_pin` suffix
- **Feature modules** follow a consistent pattern: a struct for state, `create_*`/`destroy_*` functions, and a `create_application_*` function that parses Kconfig strings
- **Conditional compilation** via `#if CONFIG_APP_*_ENABLED` guards in both headers and source
- **No dynamic endpoint creation** after startup - all endpoints are created during `app_main()`
- **Matter callbacks** are routed through `app_matter.cpp` which delegates to feature-specific handlers
- **Output pins** are optional on most features - they mirror state to a GPIO (e.g. for LEDs/relays)
- **Static storage** - each feature module uses file-scoped static arrays allocated at startup based on parsed config count

## Dependencies

Managed via ESP-IDF component registry (`idf_component.yml`):
- `espressif/button` (v4) - GPIO button handling with debounce
- `espressif/cmake_utilities` (v1) - Only for ESP32-C2

All other dependencies come from ESP-Matter SDK.
