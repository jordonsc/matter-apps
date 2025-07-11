Matter Applications
===================
This repo contains a collection of simple ESP32-based Matter devices.

Matter Certification
--------------------
These devices are configured to use the testing Matter certification by default. This will lead to the device 
initially showing as a "TEST DEVICE" after commissioning (it can then be renamed).

To change this, or the credentials required (QR/manual code) to commission, you need a proper Matter certification.

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

ESP IDF
-------

	git clone --recursive https://github.com/espressif/esp-idf.git
	cd esp-idf
	git checkout v5.4.1
	git submodule update --init --recursive
	./install.sh
	. ./export.sh
	cd ..

ESP Matter
----------
	
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
    cd <app-name>
	
    # Configure new target (only do once, or if you delete the `build` directory)
	idf.py set-target esp32s3		# replace with correct MCU

	# Build & flash
	idf.py build 
	idf.py -p /dev/ttyACM0 flash monitor


Matter Console
--------------
When the `monitor` command commands online, you actually have a serial command console to the device. Some useful
commands:

	matter config
	matter onboardingcodes ble

Troubleshooting
---------------
If a build failed, it will refuse to a do a fullclean. Fix this by deleting the build directory:

    rm -rf build/

Completely wipe the device including NVS partitions (good way to unpair):

    esptool.py -p /dev/ttyACM0 erase_flash
    
    # Re-flash:
    idf.py -p /dev/ttyACM0 flash monitor

Menu Config
-----------
Out of the box, you shouldn't need to run `menuconfig`. But this `idf.py` command allows you configure everything that
can be built into a device build.

The result will be a `sdkconfig` file created. If you need this committed to the git project, however, you need to 
move changed settings into the `sdkconfig.defaults` file (or respective defaults target).

    idf.py menuconfig
