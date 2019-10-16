piksi_firmware_private
==============

[![Build status][1]][2]

Firmware for the Swift Navigation Piksi GPS Receiver.

Checking Out Submodules
=========================

Several dependencies are submodules of this git repository.
Check them out using:

	git submodule update --init --recursive

Remember to run `git submodule update` after pulling in the latest changes to
ensure all the submodules are in sync.

Development Environment Setup
=============================
Cross Compiling Toolchain
-------------------------
We currently use the ARM GCC Embedded Toolchain, version 6-2017-q2-update, released by ARM. We mirror the correct version for our most common dev environments:
 * (64-bit x86 Linux)[https://github.com/swift-nav/swift-toolchains/releases/download/pfwp/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2]
 * (64-bit x86 macOS)[https://github.com/swift-nav/swift-toolchains/releases/download/pfwp_mac_toolchain/gcc-arm-none-eabi-6-2017-q2-update-mac.tar.bz2]
Other environments will have to download the packages from the ARM Developer website. Make sure to download the correct version, otherwise your local build may differ from the canonical build.

On Unix systems it is recomended to extract the toolchain to the `~/gcc-arm-none-eabi/` directory. Here is an example tar command to achieve this: `tar -x -C ~/gcc-arm-none-eabi/ --strip-components=1 <tarbal_name_here>`

Once extracted add the `bin` directory to your `PATH` environment variable.

macOS Packages
--------------
We maintain a set of homebrew formulas that install all of the dependencies. To install then simply run

	brew tap swift-nav/homebrew-swift-devs
	brew install piksi-firmware-toolchain

Ubuntu Packages
---------------
The following command installs the required packages for development:

	sudo apt-get install git build-essential cmake

Building
========
To build the firmware run the following command from the root of the repository once the development environment has been set up and all of the submodules have been updated.

	make PIKSI_HW=v3 PIKSI_REV=prod

Building With Docker
--------------------
While building in your native environment is usually the easiest, the container described by the `Dockerfile` is the canonical build environment. To build and run the docker container run this command from the root of the repository.

	docker-compose run piksi_firmware

Once the container is started you can follow the above directions to build the firmware.

[1]: https://travis-ci.com/swift-nav/piksi_firmware_private.svg?token=qpdcpHVrbHsVtRxV2VHR&branch=master
[2]: https://travis-ci.com/swift-nav/piksi_firmware_private
