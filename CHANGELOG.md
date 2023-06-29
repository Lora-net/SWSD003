# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.0] - UNRELEASED

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
