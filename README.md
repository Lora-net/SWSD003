# LR11xx SDK

The LR11xx SDK contains several simple examples for LR11xx transceivers.

## Examples

### Radio

| Name                 | Description                                                     | Documentation                                 |
| -------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| CAD                  | Perform a Channel Activity Detection (CAD) - LoRa only          | [README](apps/cad/README.md)                  |
| PER                  | Perform a Packet Error Rate (PER) test - both Tx and Rx roles   | [README](apps/per/README.md)                  |
| Ping pong            | Launch an exchange between two devices                          | [README](apps/ping-pong/README.md)            |
| Sprectral scan       | Get inst-RSSI values in RX mode to form a heat map              | [README](apps/spectral_scan/README.md)        |
| Spectrum display     | Get inst-RSSI values in RX mode to form a dyamic spectrum curve | [README](apps/spectrum_display/README.md)     |
| Tx continuous wave   | Configure the chip to transmit a single tone                    | [README](apps/tx_cw/README.md)                |
| Tx infinite preamble | Configure the chip to transmit an infinite preamble             | [README](apps/tx_infinite_preamble/README.md) |

A demonstration of the LR-FHSS capability of the chip can be found [here](https://github.com/Lora-net/SWDM001).

### Geolocation

| Name                   | Description                   | Documentation                 |
| ---------------------- | ----------------------------- | ----------------------------- |
| GNSS - Assisted scan   | Perform GNSS assisted scans   | [README](apps/gnss/README.md) |
| GNSS - Autonomous scan | Perform GNSS autonomous scans | [README](apps/gnss/README.md) |
| Wi-Fi passive scan     | Perform Wi-Fi passive scans   | [README](apps/wifi/README.md) |

## Configuration

Each example has its own set of parameters - see `apps/<example>/main_<example>.h`.

There is also [a common configuration file](apps/common/apps_configuration.h) where parameters can be set, among which:

* Packet type
* RF frequency
* Output power
* Packet and modulation parameters for different modulations

## Requirements

### Supported boards

This SDK is developed on the ST Microeletronic [NUCLEO-L476RG development board](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html)

### Supported shields

The list of compatible Semtech LR1110 shields is:

| Shield       | PCB                                                   | Frequency matching |
| ------------ | ----------------------------------------------------- | ------------------ |
| LR1110MB1DIS | PCB_E655V01A - GNSS with LNA for Passive GNSS Antenna | 868/915MHz         |
| LR1110MB1DJS | PCB_E656V01A - GNSS without LNA                       | 868/915MHz         |
| LR1110MB1GIS | PCB_E655V01A - GNSS with LNA for Passive GNSS Antenna | 490MHz             |
| LR1110MB1GJS | PCB_E656V01A - GNSS without LNA                       | 490MHz             |

The list of compatible Semtech LR1120 shields is:

| Shield       | PCB                                                   | Frequency matching |
| ------------ | ----------------------------------------------------- | ------------------ |
| LR1120MB1DIS | PCB_E655V01A - GNSS with LNA for Passive GNSS Antenna | 868/915MHz         |
| LR1120MB1DJS | PCB_E656V01A - GNSS without LNA                       | 868/915MHz         |
| LR1120MB1GIS | PCB_E655V01A - GNSS with LNA for Passive GNSS Antenna | 490MHz             |
| LR1120MB1GJS | PCB_E656V01A - GNSS without LNA                       | 490MHz             |

### Firmware

This SDK requires the transceiver to run the following version

* LR1110: firmware version ([0x0307](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1110/transceiver))
* LR1120: firmware version ([0x0101](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1120/transceiver))

To update the transceiver with the desired firmware version, please use [the updater tool application](https://github.com/Lora-net/lr1110_updater_tool/).

### Toolchain

Each example can be compiled with the following toolchains:

* [Keil MDK ARM](https://www2.keil.com/mdk5) - Keil project file available in `apps/<example>/MDK-ARM/`
* [GNU Arm Embedded toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) - makefile available in `apps/<example>/makefile/`

## Getting started

### Configure

Before starting to build an example, check the parameters in both the common and the example-specific configuration files.

### Build

#### Keil MDK ARM

Each example is delivered with a Keil project file - see `apps/<example>/MDK-ARM/lr11xx-sdk_<example>.uvprojx`.

To build a project:

1. Launch Keil IDE
2. Open the project file
3. Select the target
4. Compile

Each project has different targets ([Keil manual](https://www.keil.com/support/man/docs/uv4/uv4_ca_projtargfilegr.htm)), each one allowing to choose the shield the example is compiled for.

The name of the targets is taken from the column `shield` of the table available [here](#supported-shields).

#### GNU Arm embedded toolchain

Examples are built from their respective subfolder in the `apps` directory. For instance, the makefile for the `per` example is available in `apps/per/makefile/Makefile`.

Build settings, compile time and configuration options are specified in the project's Makefile.

The output files of the build process are stored in the `build` folder with firmware binary file having the same name as the project with a .bin extension.

Here are the parameters available at compile time:

| Parameter    | Description                              | Default value |
| ------------ | ---------------------------------------- | ------------- |
| TARGET_BOARD | Board for which the example is compiled  | NUCLEO_L476RG |
| RADIO_SHIELD | Shield for which the example is compiled | LR1120MB1DIS  |

For instance, to build the project `per` with LR1110MB1GJS shield:

To build a project, simply run make

```shell
$ cd $SDK_FOLDER/apps/per/makefile
$ make RADIO_SHIELD=LR1110MB1GJS
```

### Load

After a project is built, it can be loaded onto a device.

There are multiple ways to do it, among which:

* Drag and drop the binary file to the USB drive listed by our OS - usually shows up as `NODE_L476RG`.
* Load it through the Keil IDE

### View debug output

On the NUCLEO-L476RG development board, the firmware prints debug information to the UART that is connected via the ST-LINK to the host computer. The configuration is 921600/8-N-1:

* On Linux, this device usually shows up as `/dev/ttyACM0`
* On Windows, the port can be obtained from the device manager

For instance, using stty on Linux with a device available in `/dev/ttyACM0`:

```shell
$ stty -echo raw speed 921600 < /dev/ttyACM0 && cat /dev/ttyACM0
```
