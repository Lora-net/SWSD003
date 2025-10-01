# RF certification example

# ETSI FCC ARIB demo

This sample demonstrates how to test a device against ETSI (ETSI EN300.220), FCC (FCC Part 15.247), and ARIB (ARIB STD-108) certifications in the SUB-GHZ spectrum.

# Disclaimer
This demo is not officially endorsed or certified by the ETSI or FCC or ARIB committes.
The software is provided "as is", without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and noninfringement. In no event shall Semtech be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with the software or the use or other dealings in the software.

# Getting started
NOTE: if experiencing stack overflows, please increase the stack size in the linker file.

# Software flow
The software operates the radio on different frequencies. It keeps operating with one frequency until the user button is pressed.

Button description:

| Board                     | User button                                                               |
| ---------------------------- | ---------------------------------------------------------------------- |
| Nucleo L476RG | Blue button next to the black reset button                                   |

# Radio settings
Radio settings are automatically assigned to the radio for the specified test. There are different radio settings for each certification and each sub-tests.

Define the region by defining the correct region REGION_ETSI, or REGION_FCC or REGION_ARIB. The radio can be put in TX and RX mode or in RX mode only by defining MODE_TX_RX or MODE_RX_ONLY respectively.

The following defines can be set/unset to configure the software behaviour, in file [`main_rf_certification.h`](main_rf_certification.h)

| Constant              | Comment                                  |
| --------------------- | ----------------------------------------- |
| `REGION_ETSI` | Define region ETSI as certification region |
| `REGION_FCC` | Define region FCC as certification region |
| `REGION_ARIB` | Define region ARIB as certification region |
| `ENABLE_NOTIFICATIONS` | Enable sending notifications to inform receivers about the current step |
| `MODE_TX_RX` | Software is set in TX/RX mode, used for certification |
| `MODE_RX_ONLY` | Software is set in RX only mode, used for listening to notifications and perform PER measurement |

## MODE_TX_RX
This is the "main" mode for the tests, where the radio can perform the tx and rx certification tests.

## MODE_RX_ONLY
This is the mode used to act as receiver, both for receiving the notification sent every time the example performs a new test, or when performing the PER, packet error rate, (ETSI only) the radio will act as the receiver for such test.

## Notifier mode
Every time the radio performs a test, it will inform a receiver about the current tests step being executed. An additional device can be configured with RX_ONLY define to listen to these notifications.

# Certification details
Every certification standard has its own peculiarities, below find the most important ones.

## Supported Territories
| Territory | Header |
| ---------| --------|
| ETSI | [`etsi.h`](territories/etsi.h)
| FCC | [`fcc.h`](territories/fcc.h)
| ARIB | [`arib.h`](territories/arib.h)

### ETSI
In the example, the main LoRaWAN frequencies are tested with 14 dBm output. It is possible to test the 15dBm output by using the provided EXPECTED_PWR_15_DBM define.
The second part of the test is the packet error rate test. Please set another device in MODE_RX_ONLY to perform this test.

### FCC
Duty cycle (TON / (TON + TOFF))*100% of the transmissions should be > 98%. In the sweep test, the maximum time occupied for each channel is less than 400ms.

### ARIB
The transceiver will wait that for the channel rssi to be lower than -80 dBm for at least 1ms before transmitting (a time sufficient to have stable rssi measurements).