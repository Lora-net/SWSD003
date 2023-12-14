# SX126X SDK PER sample code

## Description

The sample code will be used to perform PER test for both LoRa and FSK modem tests. Define macro `PACKET_TYPE` to `SX126X_PKT_TYPE_LORA` or `SX126X_PKT_TYPE_GFSK` (in file [`../../common/apps_configuration.h`](../../common/apps_configuration.h)) to enable each modem in the test.

In PER test, the devices can be set as a TX device or a RX device.
By default the makefile provided in this example will build one binary of each kind named respectively `per_transmitter.bin` and `per_receiver.bin`. You can choose to build independently either of them by specifying the target `transmitter_target` or `receiver_target`

To set a device as RX device mode, load per_receiver.bin. The application will automatically start PER test reception to receive packets of `NB_FRAME` times.
To set a device as TX device, load per_transmitter.bin and it will send packets endlessly after startup.

### Payload setting

The first byte in payload is reserved for a rolling counter, which will be used to detect totally missed packets in PER statistics. Keeping the counter in the first byte will allow the minimum payload size. Receiver will check the rest part of payload to be sure that received packets are not unwanted ones. `per_msg` is an array to hold the content of this part of payload. If `PAYLOAD_LENGTH` (defined in [`../../common/apps_configuration.h`](../../common/apps_configuration.h)) is no longer than 1, there will be no payload check as payload will only has the counter.

## Configuration

Several parameters can be updated in [`../../common/apps_configuration.h`](../../common/apps_configuration.h) header file, refer to [`../../common/README.md`](../../common/README.md) for more details.

Several parameters can be updated in `main_per.h` header file:

| Constant              | Comments                                  | Possible values               | Default value |
| --------------------- | ----------------------------------------- | ----------------------------- | ------------- |
| `RX_TIMEOUT_VALUE`    | Timeout value for reception               | Any value fitting in uint32_t | 1000          |
| `TX_TO_TX_DELAY_IN_MS`| Time delay between 2 transmitting packets | Any value fitting in int32_t  | 200           |
| `NB_FRAME`            | Amount of packets will be tested for PER  | Any value fitting in uint16_t | 20            |

When compiling with arm-none-eabi-gcc toolchain, all these constant are configurable through command line with the EXTRAFLAGS.
See main [README](../../../README.md).
