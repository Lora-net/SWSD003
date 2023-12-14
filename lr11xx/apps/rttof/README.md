# LR11XX RTToF (ranging) sample code

## Description

The sample code is used to perform a sub-GHz band LoRa based Round-Trip Time of Flight (ranging) measurement between two devices : a RTToF manager and a RTToF subordinate.
Define the macro `RTTOF_DEVICE_MODE` in file [`main_rttof.h`](main_rttof.h) to determine whether the device operates as a RTToF manager (sends RTToF requests) or as a RTToF subordinate (processes RTToF requests and answers by sending RTToF responses).

This example can only be used with LR1110 and LR1120.

## Configuration

Several parameters can be updated in [`../../common/apps_configuration.h`](../../common/apps_configuration.h) header file, refer to [`../../common/README.md`](../../common/README.md) for more details.
Define the macros `LORA_BANDWIDTH`, `LORA_SPREADING_FACTOR` and `RF_FREQ_IN_HZ` to reflect the desired physical layer configuration.
Bandwidth shall be chosen among `LR11XX_RADIO_LORA_BW_125`, `LR11XX_RADIO_LORA_BW_250` and `LR11XX_RADIO_LORA_BW_500`.
Spreading factors SF5 to SF12 are all valid configurations.
Note also that `PACKET_TYPE` and FSK related configuration items have no effect on this sample code.

Furthermore, the application defines the following parameters in the [`main_rttof.h`](main_rttof.h) header file:

| Constant                     | Comments                                                               |
| ---------------------------- | ---------------------------------------------------------------------- |
| `RTTOF_DEVICE_MODE`          | RTToF device mode (manager, subordinate)                               |
| `RTTOF_ADDRESS`              | Subordinate device's RTToF address                                     |
| `RESPONSE_SYMBOLS_COUNT`     | Number of symbols contained in the subordinate's RTToF response packet |
| `MANAGER_TX_RX_TIMEOUT_MS`   | Manager-side RTToF timeout (waiting for subordinate response)          |
| `MANAGER_RTTOF_SLEEP_PERIOD` | Manager-side sleep period between two RTToF executions                 |
