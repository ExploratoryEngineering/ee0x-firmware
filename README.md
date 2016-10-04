# EE-0x firmware

The EE-02 board has a nRF52 SOC with a Semtech Sx1276 chip with a chip antenna.

The EE-03 board is built around the EE-02 board and includes the following additional chips.
* GPS (currently G-Top Firefly X1) with a chip antenna
* IMU (LSM9DS1) IMU
* TPS22994 power control chip

The firmware also works on a standard nRF52 DK with a Semtech Sx1276 shield, a GPS connected
to the UART pins and the SparkFun LSM9DS1 breakout board.

## Building the firmware

Project with nRF52 SDK and gcc toolchain, You'll need both the ARM gcc
compiler and the nRF52 SDK to build this.

If you are working on OS X you can download the ARM compiler via Homebrew:

    $ brew tap osx-cross/arm
    $ brew install arm-gcc-bin

Download the nRF52 11.x SDK from http://www.nordicsemi.com/eng/nordic/download_resource/54291/47/66397134
and install it into a suitable location. Update the paths at the top of the makefile to point to
the correct locations (particularly `GNU_INSTALL_VERSION`, `FLASH_DRIVE` and `NRF32_SDK_ROOT`), type `make` to build the project and `make deploy` to
deploy it to the nRF52 board (assuming you have connected it to your computer).

This is built using the nRF52 SDK 11.x. Support for version 12.x is due Real Soon Now.

## Debugging

Debug output is written using NRF_LOG et al and the makefile is set up to use the RTT logger. Download
the JLink tools at https://www.segger.com/downloads/jlink/JLink_MacOSX_V512h.pkg

Launch the JLinkExe in one terminal:

    $ JLinkExe -if swd -device nrf52 -speed 4000

...then type `connect` to connect to the device. Launch `JLinkRTTClient` in another terminal to view the
output. Note that the `JLinkExe` command is a bit flaky when you redeploy so you might have to retry the connection.

## Future expansion

This is by no means complete. The biggest missing piece is provisioning of devices. Right now the devices are
provisioned manually through hard-coded values. Fortunately the nRF52 has support for BLE so devices can be
provisioned through a mobile phone.

AES-128 encryption and decryption is done through software but the nRF52 chip has an AES encryption available.

## Licensing

The portions written by Telenor Digital uses an Apache License 2.0

Parts of this firmware includes software from other parties:
* LSM99S1 driver is adapted from source code written by SparkFun Electronics Ltd. They provide a really nice breakout
  board for the LSM9DS1 chip; highly recommended. The original source is released under a Creative Commons Share-alike 3.0 license.
  The original source can be found at https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
* LMiC library (aka "LoRa WAN in C" is written by IBM Research in Zurich with adaptations by Semtech. The Semtech portions of the
  library is rewritten to pure C. The LMiC library can be found at https://www.research.ibm.com/labs/zurich/ics/lrsc/lmic.html
  and the Semtech-adapted version is available from the mBed site: https://developer.mbed.org/teams/Semtech/code/LMiC/
* The radio driver in LMiC is adapted from the Semtech SX1276Lib driver available from https://developer.mbed.org/teams/Semtech/code/SX1276Lib/
* Keen readers might recognize parts of the battery code from the nRF52 SDK. It is no coincidence.

