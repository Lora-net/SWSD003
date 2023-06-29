# Apps common Radio Configuration

Several parameters can be updated in [`./apps_configuration.h`](./apps_configuration.h) header file:

| Constant                | Comments                                                                                                 |
| ----------------------- | -------------------------------------------------------------------------------------------------------- |
| `PACKET_TYPE`           | Packet type, will be LoRa or GFSK                                                                        |
| `RF_FREQ_IN_HZ`         | Frequency on which packets will be sent or received                                                      |
| `TX_OUTPUT_POWER_DBM`   | Power level of packets sent                                                                              |
| `FALLBACK_MODE`         | Fallback mode setting                                                                                    |
| `ENABLE_RX_BOOST_MODE`  | RX boost mode setting                                                                                    |
| `PAYLOAD_LENGTH`        | Packet payload length of both LoRa and GFSK modem                                                        |
| `LORA_BANDWIDTH`        | Bandwidth of LoRa packets                                                                                |
| `LORA_SPREADING_FACTOR` | Spreading factor of LoRa packets                                                                         |
| `LORA_CODING_RATE`      | Code rate of LoRa packets                                                                                |
| `LORA_PREAMBLE_LENGTH`  | Preamble length in LoRa packets                                                                          |
| `LORA_PKT_LEN_MODE`     | Header type, explicit (default) or implict header                                                        |
| `LORA_IQ`               | I and Q inverted or not                                                                                  |
| `LORA_CRC`              | CRC on or off                                                                                            |
| `LORA_SYNCWORD`         | Syncword used of LoRa packets                                                                            |
| `FSK_FDEV`              | Frequency deviation for GFSK modulation                                                                  |
| `FSK_BITRATE`           | BitRate for GFSK packets                                                                                 |
| `FSK_BANDWIDTH`         | Bandwidth for GFSK packets                                                                               |
| `FSK_PULSE_SHAPE`       | Defines the filtering applied to GFSK packets                                                            |
| `FSK_PREAMBLE_LENGTH`   | Length of preamble in GFSK packets                                                                       |
| `FSK_PREAMBLE_DETECTOR` | Length of preamble detector of GFSK packets                                                              |
| `FSK_SYNCWORD_LENGTH`   | Syncword length for GFSK packets                                                                         |
| `FSK_ADDRESS_FILTERING` | Option for address filtering                                                                             |
| `FSK_HEADER_TYPE`       | GFSK packet type, fixed payload length or variable                                                       |
| `FSK_CRC_TYPE`          | Option for GFSK packet CRC                                                                               |
| `FSK_DC_FREE`           | Option to enable whitening on GFSK packets                                                               |
| `gfsk_sync_word`        | Syncword for GFSK packets                                                                                |
| `FSK_WHITENING_SEED`    | Whitening seed for GFSK packets, needs `FSK_DC_FREE` enabled to be effective                             |
| `FSK_CRC_SEED`          | CRC seed for GFSK packets, needs `FSK_CRC_TYPE` set on to be effective                                   |
| `FSK_CRC_POLYNOMIAL`    | CRC polynomial for GFSK packets, needs `FSK_CRC_TYPE` set to corresponding mode                          |
| `FSK_NODE_ADDRESS`      | Node address for GFSK packets filtering, needs `FSK_ADDRESS_FILTERING` to set to corresponding mode      |
| `FSK_BROADCAST_ADDRESS` | Broadcast address for GFSK packets filtering, needs `FSK_ADDRESS_FILTERING` to set to corresponding mode |
| `SIGFOX_RC`             | Sigfox RC mode - can be 1 or 2                                                                           |
