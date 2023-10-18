# LR11XX RTToF (ranging) sample code

## Description

The sample code will be used to perform a sub-GHz band LoRa based Round-Trip Time of Flight (ranging) measurement between two devices - ranging manager and ranging subordinate. Define the macro `RANGING_DEVICE_MODE` in file [`main_ranging.h`](main_ranging.h) to determine whether the device operates as a ranging manager (sends ranging requests) or as a ranging subordinate (processes ranging requests and answers by sending ranging responses).

This example can be used only with LR1110 and LR1120.

## Configuration

Several parameters can be updated in [`../common/apps_configuration.h`](../common/apps_configuration.h) header file, refer to [`../common/README.md`](../common/README.md) for more details. Define the macros `LORA_BANDWIDTH`, `LORA_SPREADING_FACTOR` and `RF_FREQ_IN_HZ` to reflect the desired physical layer configuration. The bandwidth must be configured to one of `LR11XX_RADIO_LORA_BW_125`, `LR11XX_RADIO_LORA_BW_250`, or `LR11XX_RADIO_LORA_BW_500`. Spreading factors SF5 to SF12 are valid configurations.
Note also that `PACKET_TYPE` and FSK related configuration items have no effect on this sample code.

Furthermore, the application defines the following parameters in the [`main_ranging.h`](main_ranging.h) header file:

| Constant                       | Comments                                                                 |
| ------------------------------ | ------------------------------------------------------------------------ |
| `RANGING_DEVICE_MODE`          | Ranging device mode (manager, subordinate)                               |
| `RANGING_ADDRESS`              | Subordinate device's ranging address                                     |
| `RESPONSE_SYMBOLS_COUNT`       | Number of symbols contained in the subordinate's ranging response packet |
| `MANAGER_TX_RX_TIMEOUT_MS`     | Manager-side ranging timeout (waiting for subordinate response)          |
| `MANAGER_RANGING_SLEEP_PERIOD` | Manager-side sleep period between two ranging executions                 |
