# SX126x SDK

The SX126x SDK contains several simple examples for SX126x transceivers.

## Examples

| Name                 | Description                                                     | Documentation                                 |
| -------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| CAD                  | Perform a Channel Activity Detection (CAD) - LoRa only          | [README](sx126x/apps/cad/README.md)                  |
| PER                  | Perform a Packet Error Rate (PER) test - both Tx and Rx roles   | [README](sx126x/apps/per/README.md)                  |
| Ping pong            | Launch an exchange between two devices                          | [README](sx126x/apps/ping_pong/README.md)            |
| Sprectral scan       | Get inst-RSSI values in RX mode to form a heat map              | [README](sx126x/apps/spectral_scan/README.md)        |
| Spectrum display     | Get inst-RSSI values in RX mode to form a dyamic spectrum curve | [README](sx126x/apps/spectrum_display/README.md)     |
| Tx continuous wave   | Configure the chip to transmit a single tone                    | [README](sx126x/apps/tx_cw/README.md)                |
| Tx infinite preamble | Configure the chip to transmit an infinite preamble             | [README](sx126x/apps/tx_infinite_preamble/README.md) |

A demonstration of the LR-FHSS capability of the chip can be found [here](https://github.com/Lora-net/SWDM001).

## Configuration

Each example has its own set of parameters - see `sx126x/apps/<example>/main_<example>.h`.

There is also a common configuration file `sx126x/common/apps_configuration.h` where parameters can be set, among which:

* Packet type
* RF frequency
* Output power
* Packet and modulation parameters for different modulations

See details in the common [README](sx126x/common/README.md)

## Requirements

### Supported boards

This SDK is developed on the ST Microeletronic [NUCLEO-L476RG development board](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html)

### Supported shields

The list of compatible Semtech SX1261 shields is:

|    Shield    |   PCB    | Frequency matching |
| ------------ | -------- | ------------------ |
| SX1261MB1BAS | E406v03a |        868         |
| SX1261MB1CAS | E449V01A |        923         |
| SX1261MB2BAS | E498V01A |        868         |

The list of compatible Semtech SX1262 shields is:

|    Shield    |   PCB    | Frequency matching |
| ------------ | -------- | ------------------ |
| SX1262MB1CAS | E428V03A |        915         |
| SX1262MB1CBS | E449V01A |        923         |
| SX1262MB1DAS | E449V01A |        866         |
| SX1262MB1PAS | E449V01A |      923/915       |
| SX1262MB2CAS | E499V01B |        915         |

The list of compatible Semtech SX1268 shields is:

|    Shield    |   PCB    | Frequency matching |
| ------------ | -------- | ------------------ |
| SX1268MB1GAS | E512V01A |        490         |
