# SX126x shields

## List

The list of compatible Semtech SX1261 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                   |
| ------------ | -------- | ------------------ | --------------------------------- |
| SX1261MB1BAS | E406V03A | 868 MHz            | XTAL / RF switch / Rx and Tx LEDs |
| SX1261MB1CAS | E449V01A | 923 MHz            | TCXO / RF switch / Rx and Tx LEDs |
| SX1261MB2BAS | E498V01A | 868 MHz            | XTAL / RF switch                  |

The list of compatible Semtech SX1262 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                   |
| ------------ | -------- | ------------------ | --------------------------------- |
| SX1262MB1CAS | E428V03A | 915 MHz            | XTAL / RF switch / Rx and Tx LEDs |
| SX1262MB1CBS | E449V01A | 923 MHz            | TCXO / RF switch / Rx and Tx LEDs |
| SX1262MB1DAS | E449V01A | 866 MHz            | TCXO / RF switch / Rx and Tx LEDs |
| SX1262MB1PAS | E449V01A | 923/915 MHz        | TCXO / RF switch / Rx and Tx LEDs |
| SX1262MB2CAS | E499V01B | 915 MHz            | XTAL / RF switch                  |

The list of compatible Semtech SX1268 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                   |
| ------------ | -------- | ------------------ | --------------------------------- |
| SX1268MB1GAS | E512V01A | 490 MHz            | XTAL / RF switch / Rx and Tx LEDs |

## GPIO

There are several GPIOs that the application has to handle.

Some are related to the chip and are mandatory:

* Chip select
* Busy
* Interrupt
* Reset

Some are related to the shield:

* Antenna switch
* Led RX
* Led Tx
