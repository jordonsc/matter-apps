Matter Applications
===================
This repo contains a collection of simple ESP32-based Matter devices.

Matter Certification
--------------------
These devices are configured to use the testing Matter certification by default. This will lead to the device 
initially showing as a "TEST DEVICE" after commissioning (it can then be renamed).

To change this, or the credentials required (QR/manual code) to commission, you need a proper Matter certification.

Reading Materials
-----------------
* [Matter Specifications](https://handbook.buildwithmatter.com/specification/)
* [Kconfig Example](https://github.com/espressif/esp-idf/blob/master/examples/common_components/protocol_examples_common/Kconfig.projbuild)

Getting Started
---------------
You'll need a few system packages available, and then both of the ESP-IDF SDK and the ESP-Matter SDK. Both follow
a similar concept: clone the repo, run an installer to bootstrap your environment and then when working, you need
to export an "export" file in both repos.

The `init` file in this project root will export both SDKs, once you've done the initial installs:

    # Load the ESP-IDF and ESP-Matter SDKs:
    . ./init

### System Packages
For Linux (Ubuntu) users:

	sudo apt install -y cmake gn ninja-build python3.13-venv pkg-config libssl-dev libglib2.0-dev
	sudo usermod -aG dialout `whoami`
	newgrp dialout

### ESP IDF

	git clone --recursive https://github.com/espressif/esp-idf.git
	cd esp-idf
	git checkout v5.4.1
	git submodule update --init --recursive
	./install.sh
	. ./export.sh
	cd ..

### ESP Matter
	
	git clone --recursive https://github.com/espressif/esp-matter.git
	git checkout release/v1.4
	cd ./connectedhomeip/connectedhomeip
	./scripts/checkout_submodules.py --platform esp32 linux --shallow
	cd ../..
	./install.sh
	. ./export.sh
	cd ..

Building Apps
-------------

    # On new terminal - export both SDKs:
    . ./init
    cd omni
	
    # Configure new target (only do once, or if you delete the `build` directory)
	idf.py set-target esp32s3		# replace with correct MCU

	# Configure Matter endpoints (see below):
	idf.py menuconfig

	# Build & flash
	idf.py build 
	idf.py -p /dev/ttyACM0 flash monitor

Applications
------------
### Omni

* [Omni Application documentation](omni/README.md)

The "Omni" app is designed to be a general-purpose configurable application that allows you to quickly build a device
with switches, buttons and binary sensors on a GPIO by simply setting which device is connected to which GPIO.

The ESP-Matter SDK uses a KConfig system to generate environment variables which define your project configuration.
The default files include a working set of configuration, but to add Matter endpoints to the application, you need to
invoke the `menuconfig` system and set pin configuration, etc in the `Omni` menu.

    idf.py menuconfig

Matter Console
--------------
When the `monitor` command comes online, you actually have a serial command console to the device. Some useful
commands:

	matter config
	matter onboardingcodes ble

> Note that until you've completed Matter certification, you will need to use the TESTING DEVICE onboarding codes.

Features
--------
A set of Matter endpoints has implementations under `common/features` and can be easily enabled or imported for any
application. Some of these endpoints are similar, but behave differently when connected to a control plane.

### Button (Generic Switch)
Ideal for using a physical momentary switch that will send events when pressed, but cannot be triggered from the
control plane. Think of it as a read-only endpoint in controllers.

A Matter "Generic Switch" has two modes: momentary and latching. 

The momentary is the most typical use: you push a button, it sends an event. You can additionally support double-clicks
and long-presses. 

The latching type is akin to a flip switch - you can turn it on or off, but is controlled only by the physical device, 
never by the control plane. This behaves oddly in _Home Assistant_, as for the "Generic Switch" endpoint it is only 
listening for events. The event type is "latched", you will need additional conditions to also inspect the 
`NewPosition` property to determine which way it latched.

### On/Off
This is a simple switch but unlike the button, it's designed to be controllable via the control plane. This allows
for both the controller and the device to toggle the endpoint's binary state.

For this reason, it's important NOT to use a physical flip switch on an On/Off endpoint, for the controller might 
toggle the endpoint itself and you won't be able to toggle the physical switch electronically. 

A push-button is appropriate, each push can toggle the state of the On/Off, or it can be toggled by the controller.

### Sensor
The sensor feature provides binary sensors that are read-only to the control plane and describe the state of physical 
sensors connected to GPIO pins.

Two types of binary sensors are supported:

**Occupancy Sensor** - Uses the Matter OccupancySensing cluster, which is the standard binary sensor type in the 
Matter specification. This is ideal for motion detectors, door/window sensors, or any physical presence detection.

**Generic Sensor** - Uses the Matter BooleanState cluster, providing a more flexible binary sensor that can represent 
any on/off state. This is useful for custom sensors or binary inputs that don't fit the occupancy model.

Both sensor types support inverted logic for flexibility with different sensor wiring configurations.

Configuration examples:
- `O34` - Occupancy sensor on GPIO 34
- `OI34` - Occupancy sensor on GPIO 34 with inverted logic
- `G12` - Generic sensor (BooleanState) on GPIO 12  
- `GI12` - Generic sensor on GPIO 12 with inverted logic

### Reset
The reset feature is important, it allows you to "factory reset" the device but suppressing a button for a configurable
period of time. This allows you to recommission the device.

When activated, the device will wipe the NVS partition and reboot. When it comes back up (only takes a second), it
will be ready for commissioning again.

This is the only way to recommission a device without reflashing it.

Troubleshooting
---------------
If a build failed, it will refuse to do a `fullclean`. Fix this by deleting the build directory:

    rm -rf build/

Completely wipe the device including NVS partitions (good way to unpair):

    esptool.py -p /dev/ttyACM0 erase_flash
    
    # Re-flash:
    idf.py -p /dev/ttyACM0 flash monitor


