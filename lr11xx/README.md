# LR11xx SDK

The LR11xx SDK contains several simple examples for LR11xx transceivers.

## Examples

| Name                                            | Description                                                                                 | Documentation                                                         |
| ----------------------------------------------- | ------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| Bluetooth Low Energy(R) Beaconing Compatibility | Demonstrate Bluetooth® Low Energy Beaconing Compatibility. Only valid for LR1110 and LR1120 | [README](apps/bluetooth_low_energy_beaconing_compatibility/README.md) |
| CAD                                             | Perform a Channel Activity Detection (CAD) - LoRa only                                      | [README](apps/cad/README.md)                                          |
| PER                                             | Perform a Packet Error Rate (PER) test - both Tx and Rx roles                               | [README](apps/per/README.md)                                          |
| Ping pong                                       | Launch an exchange between two devices                                                      | [README](apps/ping_pong/README.md)                                    |
| RTToF (Ranging)                                 | Perform Round-Trip Time of Flight (ranging) exchanges. Only valid for LR1110 and LR1120     | [README](apps/rttof/README.md)                                        |
| Sigfox                                          | Send a Sigfox-compliant uplink                                                              | [README](apps/sigfox/README.md)                                       |
| Spectral scan                                   | Get inst-RSSI values in RX mode to form a heat map                                          | [README](apps/spectral_scan/README.md)                                |
| Spectrum display                                | Get inst-RSSI values in RX mode to form a dyamic spectrum curve                             | [README](apps/spectrum_display/README.md)                             |
| Tx continuous wave                              | Configure the chip to transmit a single tone                                                | [README](apps/tx_cw/README.md)                                        |
| Tx infinite preamble                            | Configure the chip to transmit an infinite preamble                                         | [README](apps/tx_infinite_preamble/README.md)                         |

A demonstration of the LR-FHSS capability of the chip can be found [here](https://github.com/Lora-net/SWDM001).

## Configuration

Each example has its own set of parameters - see `lr11xx/apps/<example>/main_<example>.h`.

There is also [a common configuration file](common/apps_configuration.h) where parameters can be set, among which:

* Packet type
* RF frequency
* Output power
* Packet and modulation parameters for different modulations

## Requirements

### Supported boards

This SDK is developed on the ST Microeletronic [NUCLEO-L476RG development board](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html)

### Supported shields

The list of compatible Semtech LR1110 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                        |
| ------------ | -------- | ------------------ | -------------------------------------- |
| LR1110MB1DIS | E516V02B | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna |
| LR1110MB1DJS | E592V01B | 868/915 MHz        | GNSS without LNA                       |
| LR1110MB1GIS | E516V02B | 490 MHz            | GNSS with LNA for Passive GNSS Antenna |
| LR1110MB1GJS | E592V01B | 490 MHz            | GNSS without LNA                       |

The list of compatible Semtech LR1120 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                        |
| ------------ | -------- | ------------------ | -------------------------------------- |
| LR1120MB1DIS | E655V01A | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna |
| LR1120MB1DJS | E656V01A | 868/915 MHz        | GNSS without LNA                       |
| LR1120MB1GIS | E655V01A | 490 MHz            | GNSS with LNA for Passive GNSS Antenna |
| LR1120MB1GJS | E656V01A | 490 MHz            | GNSS without LNA                       |

The list of compatible Semtech LR1121 shields is:

| Shield       | PCB      | Frequency matching | Characteristics |
| ------------ | -------- | ------------------ | --------------- |
| LR1121MB1DIS | E655V01A | 868/915 MHz        | N/A             |
| LR1121MB1GIS | E655V01A | 490 MHz            | N/A             |

### Firmware

This SDK requires the transceiver to run the following version

* LR1110: firmware version ([0x0401](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1110/transceiver))
* LR1120: firmware version ([0x0201](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1120/transceiver))
* LR1121: firmware version ([0x0103](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1121/transceiver))

To update the transceiver with the desired firmware version, please use [the updater tool application](https://github.com/Lora-net/SWTL001).

### Workaround

#### High ACP (Adjacent Channel Power)

This issue has beed fixed in firmware LR1110 0x0308 and LR1120 0x0102 - it was not present in LR1121 0x0101. The associated workaround is disabled in this project starting from v2.1.0.

If one wants to re-enable the workaround, this [line](common/apps_common.mk#L33) has to be commented.
