# Generic Switch

## Switch Configuration

There are two types of switches in the example which can be configured `menuconfig`:

`idf.py menuconfig` -> Application -> Generic Switch Type

* To use latching switch, enable `GENERIC_SWITCH_TYPE_LATCHING`.
* To use a momentary switch, enable `GENERIC_SWITCH_TYPE_MOMENTARY`.

The default type is `GENERIC_SWITCH_TYPE_MOMENTARY`.

Set the GPIO pins in comma-delimited list:

    # Push button on GPIO 9
    `9`

    # Or on boot, 9 and 13 as three different endpoints:
    `0,9,13`
