# LR11XX SDK PER sample code

## Description

The sample code will be used to perform PER test for both LoRa and FSK modem tests. Define macro `PACKET_TYPE` to `LR1110_RADIO_PKT_TYPE_LORA` or `LR1110_RADIO_PKT_TYPE_GFSK` (in file [`../common/apps_configuration.h`](../common/apps_configuration.h) to enable each modem in the test.

In PER test, the devices can be set as a TX device or a RX device. There are also macros to control the compilation. To set device to RX, the macro (`RECEIVER`) has to be set to 1; to set device to TX, set it to 0. If set to RX device mode, this application will automatically start PER test reception to receive packets of `NB_FRAME` times. If set to TX device, it will send packets endlessly after startup.

### Payload setting

The first byte in payload is reserved for a rolling counter, which will be used to detect totally missed packets in PER statistics. Keeping the counter in the first byte will allow the minimum payload size. Receiver will check the rest part of payload to be sure that received packets are not unwanted ones. `per_msg` is an array to hold the content of this part of payload. If `PAYLOAD_LENGTH` (defined in [`../common/apps_configuration.h`](../common/apps_configuration.h) is no longer than 1, there will be no payload check as payload will only has the counter.

## Configuration

Several parameters can be updated in [`../common/apps_configuration.h`](../common/apps_configuration.h) header file, refer to [`../common/README.md`](../common/README.md) for more details.

Several parameters can be updated in [`main_per.h`](main_per.h) header file:

| Constant              | Comments                                  |
| --------------------- | ----------------------------------------- |
| `RECEIVER`            | RX or TX device                           |
| `RX_TIMEOUT_VALUE`    | Timeout value for reception               |
| `TX_TO_TX_DELAY_IN_MS`| Time delay between 2 transmitting packets |
| `NB_FRAME`            | Amount of packets will be tested for PER  |
