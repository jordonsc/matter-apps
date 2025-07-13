# Omni Application
This application is designed to be configurable from `menuconfig` to add any endpoint type as required. You can mix
different Matter endpoints as per your device's need.

## Switch Configuration
Switches or buttons can be configured via `menuconfig`.

`idf.py menuconfig` -> Application -> Switch

### Switch Type

* To use latching switch, enable `GENERIC_SWITCH_TYPE_LATCHING`.
* To use a momentary switch, enable `GENERIC_SWITCH_TYPE_MOMENTARY`.

The default type is `GENERIC_SWITCH_TYPE_MOMENTARY`.

### Pin List
Set the GPIO pins in comma-delimited list:

    # Push button on GPIO 9
    `9`

    # Or on boot, 9 and 13 as three different endpoints:
    `0,9,13`

### Button Responsiveness
To make a momentary button as responsive as possible, disable double-click and long-press. This will allow you to 
trigger when button has been initially depressed.

If you enable long-press, you should also enable double-click. This will give your control-plane enough context to
know which event happened, but with double-click enabled there is a short delay before concluding the click-counter,
so this option, while the most flexible, will also be less responsive.
