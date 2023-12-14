# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v2.2.0] - 2023-12-15

### Added

- LR-FHSS demo support for LR11XX and SX126X chips
- SX126X Sigfox demo support
- Bluetooth Low Energy Beaconing Compatibility example for LR1110 and LR1120
- Support for the following shields
  - LR1110MB1IPDDIS
  - LR1120MB1IPDDIS
  - LR1110MB1PIS
  - LR1120MB1PIS

### Fixed

- Remove duplicated code in configuration of LR11xx DBPSK configuration
- PA tables for LR1110MB1DxS, LR1110MB1GxS, LR1120MB1DxS, LR1120MB1GxS
- Typos in readme files
- SX126x Ping-Pong example: add handler for Rx CRC error

### Changed

- Renaming from ranging to RTToF
- LR11xx driver upgraded to v2.4.1
- Update latest firmware version definition

## [v2.1.0] - 2023-10-06

### Added

- LR11xx ranging example

## [v2.0.2] - 2023-09-05

### Removed

- Removed *targets.txt* files
- LR11xx geolocation-related examples (GNSS and Wi-Fi scan)
  - The geolocation-related examples are distributed through [SWSD004](https://github.com/Lora-net/SWSD004)

### Fixed

- Fixed Keil project files that were not compiling with error `'smtc_dbpsk.h' file not found`
- Fixed sx216x Keil project files that were not compiling due to bad path of some driver files
- Fixed broken links in README file of SX126x

## [2.0.1] - 2023-06-29

### Added

- `LR11XX_SYSTEM_IRQ_LORA_RX_TIMESTAMP` management in `apps_common_lr11xx_irq_process`

### Changed

- Updated LR11xx driver to v2.3.0
- Macro `LR11XX_DISABLE_HIGH_ACP_WORKAROUND` is now defined to disable the corresponding workaround
- Add `Sigfox` example with LR11xx
- Default `TX_OUTPUT_POWER_DBM` default value for LR11xx examples to `13 dBm`

### Removed

- LR11xx geolocation-related examples (GNSS and Wi-Fi scan)

### Added

- SX126x support
- LR1121 support
- SMTC HAL MCU - simplified MCU hardware abstration layer to ease porting on another platform
- Shield interface - library to get specificities related to each LR11xx / SX126x shield

## [2.0.0] - 2023-03-06

### Added

- SX126x support
- LR1121 support
- SMTC HAL MCU - simplified MCU hardware abstration layer to ease porting on another platform
- Shield interface - library to get specificities related to each LR11xx / SX126x shield

## [1.0.0] - 2022-04-07

### Added

- Initial version
