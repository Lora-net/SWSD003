# LR11xx shields

## List

The list of compatible Semtech LR1110 shields is:

| Shield          | PCB      | Frequency matching | Characteristics                                                                |
| --------------- | -------- | ------------------ | ------------------------------------------------------------------------------ |
| LR1110MB1DIS    | E516V02B | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna                                         |
| LR1110MB1IPDDIS | E613V02A | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna and IPD device on RF LF and RF HF paths |
| LR1110MB1PIS    | E679V02A | 923 MHz            | GNSS with LNA for Passive GNSS Antenna and SAW filter on Tx LF path            |
| LR1110MB1DJS    | E592V01B | 868/915 MHz        | GNSS without LNA                                                               |
| LR1110MB1GIS    | E516V02B | 490 MHz            | GNSS with LNA for Passive GNSS Antenna                                         |
| LR1110MB1GJS    | E592V01B | 490 MHz            | GNSS without LNA                                                               |

The list of compatible Semtech LR1120 shields is:

| Shield          | PCB      | Frequency matching | Characteristics                                                                |
| --------------- | -------- | ------------------ | ------------------------------------------------------------------------------ |
| LR1120MB1DIS    | E655V01A | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna                                         |
| LR1120MB1IPDDIS | E613V02A | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna and IPD device on RF LF and RF HF paths |
| LR1120MB1PIS    | E679V02A | 923 MHz            | GNSS with LNA for Passive GNSS Antenna and SAW filter on Tx LF path            |
| LR1120MB1DJS    | E656V01A | 868/915 MHz        | GNSS without LNA                                                               |
| LR1120MB1GIS    | E655V01A | 490 MHz            | GNSS with LNA for Passive GNSS Antenna                                         |
| LR1120MB1GJS    | E656V01A | 490 MHz            | GNSS without LNA                                                               |

The list of compatible Semtech LR1121 shields is:

| Shield       | PCB      | Frequency matching | Characteristics |
| ------------ | -------- | ------------------ | --------------- |
| LR1121MB1DIS | E655V01A | 868/915 MHz        |                 |
| LR1121MB1GIS | E655V01A | 490 MHz            |                 |

## Note concerning shields with IPDs

The shields `LR1110MB1IPDDIS` and `LR1120MB1IPDDIS` feature Integrated Passive Devices (IPDs) that help to reduce footprints.
These shields have specific Power Amplifier configuration tuning on LR11xx chip that have been evaluated to comply with FCC and ETSI regulations.

The power amplifier configuration that comply with FCC and ETSI regulations have been established only on a limited Tx power configuration.

Therefore the power amplifier configuration given in this code base returns the established FCC or ETSI compliant configuration if available, and fallback to common configuration if not found.

## Note concerning LR11x0MB1PIS shields

The power amplifier configuration provided in this repository for `LR1110MB1PIS` and `LR1120MB1PIS` shields is so that:

- if the requested power corresponds to the configuration used to pass ARIB harmonic test (+6dBm), then this one is returned;
- if the requested power is inferior to the one used during ARIB harmonic test, then the configuration returned is the same as for LR11x0MB1DxS shields;
- otherwise no configuration is returned.

The power amplifier configuration targeting +6dBm provided in this repository has been tested against ARIB harmonic requirements.
However it has not been optimized for power consumption.

The power amplifier configurations returned for power target inferior to +6dBm are the same as for LR11x0MB1DxS shields.
These configurations have not been tested for ARIB harmonic requirements.

The default BIll Of Material of LR11x0MB1PIS shields does not allow to use the High Power RF path.
Therefore the power amplifier configuration must be so that only Low Power RF path is used.
