# LR11XX SDK CAD sample code

## Description

The chip performs the CAD operation in LoRa. Define macro `PACKET_TYPE` to `LR1110_RADIO_PKT_TYPE_LORA` (in file [`../common/apps_configuration.h`](../common/apps_configuration.h)) to use this sample code.

In CAD test, there are 3 kinds of exit mode that can be defined for different use cases. For `CAD_ONLY` mode, once done and whatever the activity on the channel, the chip goes back to STBY_RC mode. For `CAD_RX` mode, if an activity is detected, it stays in RX until a packet is detected or the timer reaches the timeout defined by `CAD_TIMOUT_MS`. For `CAD_LBT` mode, if no activity is detected, it goes to tx mode. This mode is actually a substitue of tx mode, so payload data for transmitting should be preloaded before setting the chip to `CAD_LBT` mode.

## Configuration

Several parameters can be updated in [`../common/apps_configuration.h`](../common/apps_configuration.h) header file, refer to [`../common/README.md`](../common/README.md) for more details.

Several parameters can be updated in [`main_cad.h`](main_cad.h) header file:

| Constant          | Comments                                                                      |
| ----------------- | ----------------------------------------------------------------------------- |
| `CAD_SYMBOL_NUM`  | Defines the number of symbols used for the CAD detection                      |
| `CAD_DETECT_PEAK` | Define the sensitivity of the LoRa modem when trying to correlate to symbols  |
| `CAD_DETECT_MIN`  | Minimum peak value, meant to filter out case with almost no signal or noise.  |
| `CAD_EXIT_MODE`   | Defines the action to be performed after a CAD operation                      |
| `CAD_TIMOUT_MS`   | Only used when the CAD is performed with CAD_EXIT_MODE = CAD_RX or CAD_LBT    |
