piksi_firmware_private
==============

[![Build status][1]][2]

### Note: Piksi v2 is no longer supported.
Follow [this link](https://github.com/swift-nav/piksi_firmware_private/tree/last-v2-commit) for the final Piksi v2 compatible commit.

Firmware for the Swift Navigation Piksi GPS Receiver.

Documentation available online at http://docs.swift-nav.com/piksi_firmware

Checking Out Submodules
=========================

ChibiOS, libopencm3, libsbp and starling are submodules of this git repository.
Check them out using:

	git submodule update --init --recursive

Remember to run `git submodule update` after pulling in the latest changes to
ensure all the submodules are in sync.

Build
=====
macOS
-----
First install python if you don't have it yet:
```bash
% brew install python
```
This installs Python 3; symlinks for `python` and `pip` are created in `/usr/local/opt/python/libexec/bin`
so add this dir to your PATH in ~/.bashrc, ~/.profile, or similar, and restart the shell.
```bash
export PATH="$PATH:/usr/local/opt/python/libexec/bin"
```

Now run the script that calls the ansible installer
```bash
% bash setup.sh -x install
```

Add the path for the arm-gcc compiler to ~/.bashrc, ~/.profile, or similar:
```bash
export PATH="$PATH:$HOME/gcc-arm-none-eabi/bin"
```
Run the build
```bash
% make PIKSI_HW=v3 PIKSI_REV=prod
```

Installation
============

There are a few options:

* **Normal usage**. If you're only using the Piksi console, binary
  installers (Windows and OS X) are
  [here](http://downloads.swiftnav.com/piksi_console/) and source
  for the console can be found in
  [piksi_tools](https://github.com/swift-nav/piksi_tools).

* **Development (native)**. To install dependencies for the
  development tools on your platform (OS X, Ubuntu, or Debian), run
  the setup script in this repository via `bash setup.sh -x
  install`. If you're also building the firmware, you'll need to
  checkout the submodules as well.

* **Development (VM)**. The Vagrant file is currently used for testing
  installation setup.sh, but can also be used to provision a
  development VM. To do so, you will need to download
  [VirtualBox](https://www.virtualbox.org/wiki/Downloads) and
  [Vagrant](http://www.vagrantup.com/downloads.html), and then run
  `vagrant up trusty` in this repository.

For additional details about the toolchain installation, please see
  http://docs.swift-nav.com/wiki/Piksi_Developer_Getting_Started_Guide .

[1]: https://travis-ci.com/swift-nav/piksi_firmware_private.svg?token=qpdcpHVrbHsVtRxV2VHR&branch=master
[2]: https://travis-ci.com/swift-nav/piksi_firmware_private
