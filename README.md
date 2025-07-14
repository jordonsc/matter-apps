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
The "Omni" app is designed to be a general-purpose configurable application that allows you to quickly build a device
with switches, buttons and binary sensors on a GPIO by simply setting which device is connected to which GPIO.

The ESP-Matter SDK uses a KConfig system to generate environment variables which define your project configuration.
The default files include a working set of configuration, but to add Matter endpoints to the application, you need to
invoke the `menuconfig` system and set pin configuration, etc in the `Omni` menu.

    idf.py menuconfig

Matter Console
--------------
When the `monitor` command commands online, you actually have a serial command console to the device. Some useful
commands:

	matter config
	matter onboardingcodes ble

> Note that until you've completed Matter certification, you will need to use the TESTING DEVICE onboarding codes.

Troubleshooting
---------------
If a build failed, it will refuse to a do a fullclean. Fix this by deleting the build directory:

    rm -rf build/

Completely wipe the device including NVS partitions (good way to unpair):

    esptool.py -p /dev/ttyACM0 erase_flash
    
    # Re-flash:
    idf.py -p /dev/ttyACM0 flash monitor


