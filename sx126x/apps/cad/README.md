# SX126X SDK CAD sample code

## Description

The chip performs the CAD operation in LoRa. Define macro `PACKET_TYPE` to `SX126X_PKT_TYPE_LORA` (in file (`../../common/apps_configuration.h`)) to use this sample code.

In CAD test, there are 3 kinds of exit mode that can be defined for different use cases:

- `CAD_ONLY` mode - once done and whatever the activity on the channel, the chip goes back to STBY_RC mode. 
- `CAD_RX` mode - if an activity is detected, it stays in RX until a packet is detected or the timer reaches the timeout defined by `CAD_TIMEOUT_MS`.
- `CAD_LBT` mode - if no activity is detected, it goes to tx mode. This mode is actually a substitute to tx mode, so payload data for transmitting should be preloaded before setting the chip to `CAD_LBT` mode.

By default, the provided makefile will build one binary of each kind.
Any mode can be built independently with the following target names: `cad_only`, `cad_then_receive`, `cad_then_transmitt`


## Configuration

Several parameters can be updated in `../../common/apps_configuration.h` header file, refer to `../../common/README.md` for more details.

Several parameters can be updated in `main_cad.h` header file:

| Constant          | Comments                                                                      | Possible values                             | Default value    |
| ----------------- | ----------------------------------------------------------------------------- | ------------------------------------------- | ---------------- |
| `CAD_SYMBOL_NUM`  | Defines the number of symbols used for the CAD detection                      |                           1, 2, 4, 8 or 16  |                2 |
| `CAD_DETECT_PEAK` | Define the sensitivity of the LoRa modem when trying to correlate to symbols  |                                     [22-25] |               22 |
| `CAD_DETECT_MIN`  | Minimum peak value, meant to filter out case with almost no signal or noise.  |                                         10  |               10 |
| `CAD_EXIT_MODE`   | Defines the action to be performed after a CAD operation                      | Any value of enum `sx126x_cad_exit_modes_t` | `SX126X_CAD_LBT` |
| `CAD_TIMEOUT_MS`  | Only used when the CAD is performed with CAD_EXIT_MODE = CAD_RX or CAD_LBT    | Any value that fits in `uint32_t`           |             1000 |

When compiling with arm-none-eabi-gcc toolchain, all these constant are configurable through command line with the EXTRAFLAGS.
See main [README](../../../README.md).
