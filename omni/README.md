Omni Application
================
This application is designed to be configurable from `menuconfig` to add any endpoint type as required. You can mix
different Matter endpoints as per your device's need.

Configuration Overview
----------------------
All features can be configured via `menuconfig`:

`idf.py menuconfig` -> Omni

### Available Features
- **Reset**: Factory reset functionality via GPIO button
- **Switch**: Generic Switch endpoints (momentary/latching buttons)  
- **OnOff**: On/Off devices (lights, outlets, switches)
- **Sensor**: Binary sensors (occupancy, generic)

## Switch Configuration
Switches or buttons can be configured via `menuconfig` -> Omni -> Switch.

### GPIO List Format
The GPIO list uses a space-separated format with optional output pins:

```
BUTTON_GPIO_LIST="M9 L10:12 MX8:13"
```

Where:
- `M9` - Momentary button on GPIO 9
- `L10:12` - Latching switch on GPIO 10 with output on GPIO 12
- `MX8:13` - Momentary button with double-click and long-press on GPIO 8, output on GPIO 13

### Switch Types and Features
- **L** - Latching switch
- **M** - Momentary button (default)
- **MD** - Momentary with double-click
- **ML** - Momentary with long-press  
- **MX** - Momentary with double-click and long-press

### Output Pins
Output pins are optional and specified after a colon `:`. When configured, the output pin will be:
- Set HIGH when the button is pressed (for momentary) or latched (for latching)
- Set LOW when the button is released or unlatched

## OnOff Device Configuration
OnOff devices can be configured via `menuconfig` -> Omni -> OnOff.

### GPIO List Format
```
ONOFF_GPIO_LIST="L34:12 O22 S16:9"
```

Where:
- `L34:12` - Light on GPIO 34 with output on GPIO 12
- `O22` - Outlet on GPIO 22 (no output pin)

### Device Types
- **L** - Light
- **O** - Outlet

### Output Pin Behavior
- Output pin is set HIGH when the device state is ON
- Output pin is set LOW when the device state is OFF
- State changes from both GPIO input and Matter controller commands update the output pin

## Sensor Configuration
Binary sensors can be configured via `menuconfig` -> Omni -> Sensor.

### GPIO List Format
```
SENSOR_GPIO_LIST="O34:9 GI12:13"
```

Where:
- `O34:9` - Occupancy sensor on GPIO 34 with output on GPIO 9
- `GI12:13` - Generic sensor on GPIO 12 (inverted logic) with output on GPIO 13

### Sensor Types and Options
- **O** - Occupancy sensor (OccupancySensing cluster)
- **G** - Generic sensor (BooleanState cluster)
- **I** - Inverted logic (add after sensor type, e.g., `OI` or `GI`)

### Output Pin Behavior
- Output pin is set HIGH when sensor is triggered/active
- Output pin is set LOW when sensor is not triggered/inactive
- For inverted sensors, the logic is applied before updating the output pin

## Reset Configuration
Factory reset can be configured via `menuconfig` -> Omni -> Reset.

- **RESET_GPIO_PIN**: GPIO pin for the reset button
- **RESET_HOLD_TIME**: Time in milliseconds the button must be held

## Examples

### Smart Light Switch
```
# Light switch with status LED
ONOFF_GPIO_LIST="L5:2"
```
Creates a light on GPIO 5 with an LED indicator on GPIO 2.

### Motion Sensor with LED
```
# Motion sensor with indicator LED
SENSOR_GPIO_LIST="O18:2"
```
Creates an occupancy sensor on GPIO 18 with LED on GPIO 2.

### Multi-button Panel
```
# Three buttons with individual LEDs
BUTTON_GPIO_LIST="M9:12 M10:13 ML11:14"
```
Creates three momentary buttons (GPIO 9, 10, 11) with corresponding LED outputs (GPIO 12, 13, 14).
The third button also supports long-press events.
