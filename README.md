# SWSD003

This SDK contain several simple examples for SX126x and LR11XX chip families.

## Examples

For a detailed description of the available examples and their configuration, refer to the corresponding README file depending on chip family:

- **SX126X**: [SX126X readme](sx126x/README.md)
- **LR11XX**: [LR11XX readme](lr11xx/README.md)

The readme files also provide the compatible products, along with hardware and software requirements.

## Getting started

### Configure

Before starting to build an example, check the parameters in both the common and the example-specific configuration files.
The common parameters can be found in `<chip family>/common/apps_configuration.h` while the example specific configuration file is located in the example folder. For example, the `per` example finds its configuration in `<chip_family>/apps/per/main_per.h`

### Build

In this section:
- `<chip_family>` is:
  - `sx126x` for SX126X examples
  - `lr11xx` for LR11XX examples
- `<example>`: is the name of the selected example. Refer to corresponding README file of the chip family for possible values

#### Keil MDK ARM

Each example is delivered with a Keil project file - see `<chip_family>/apps/<example>/MDK-ARM/<chip_family>-sdk_<example>.uvprojx`.

To build a project:

1. Launch Keil IDE
2. Open the project file
3. Select the target
4. Compile

Each project has different targets ([Keil manual](https://www.keil.com/support/man/docs/uv4/uv4_ca_projtargfilegr.htm)), each one allowing to choose the shield the example is compiled for.

The name of the targets is taken from the column `shield` of the supported shields table available in the chip family READMEs:
- for SX126X:  [here](sx126x/README.md#supported-shields)
- for LR11XX:  [here](lr11xx/README.md#supported-shields)

#### GNU Arm embedded toolchain

Examples are built from their respective subfolder in the `apps` directory. For instance, the makefile for the `per` example for LR11XX is available in `lr11xx/apps/per/makefile/Makefile`.

Build settings, compile time and configuration options are specified in the project's Makefile.

The output files of the build process are stored in the `build` folder with firmware binary file having the same name as the project with a .bin extension.

Here are the parameters available at compile time:

| Parameter    | Description                              | Default value                                      |
| ------------ | ---------------------------------------- | -------------------------------------------------- |
| TARGET_BOARD | Board for which the example is compiled  | NUCLEO_L476RG                                      |
| RADIO_SHIELD | Shield for which the example is compiled | **lr11xx**: LR1120MB1DIS, **sx126x**: SX1262MB1CAS |

For instance, to build the project `per` with LR1110MB1GJS shield simply run make as follows

```shell
$ cd $SDK_FOLDER/lr11xx/apps/per/makefile
$ make RADIO_SHIELD=LR1110MB1GJS
```

##### Command line configuration

Additional configuration flags can be passed from command line to compiler with `EXTRAFLAGS` argument.
This is dedicated to define macros that can be defined like the following:

```bash
$ make EXTRAFLAGS='-D<MACRO>=<VALUE>'
```

Where `<MACRO>` is the macro name to set and `<VALUE>` is the value to set for this macro.
Not all macro can be redefined through this way. Refer to the README of examples for the list of macro that can be redefined.

Note that when using the configuration on command line, `make` cannot detect a change in configuration on next build.
Therefore `make clean` must be invoked before calling a new `make` with a different configuration

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

## Requirements

Additional requirements specific to chip family are provided in the corresponding README file.

### Supported toolchains

Each example can be compiled with the following toolchains:

* [Keil MDK ARM](https://www2.keil.com/mdk5) - Keil project file available in `<chip_family>/apps/<example>/MDK-ARM/`
* [GNU Arm Embedded toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) - makefile available in `<chip_family>/apps/<example>/makefile/`

## Known limitations

### Channel Activity Detection accuracy

The Channel Activity Detection (CAD) may expose false negative or false positive detection.
The parameters for CAD configuration needs adaptation relative to context usage.
Refer to the application notes available on [LoRa Developer Portal](https://lora-developers.semtech.com/documentation/product-documents/) and [Semtech website](https://www.semtech.com/) relative to CAD performances.
