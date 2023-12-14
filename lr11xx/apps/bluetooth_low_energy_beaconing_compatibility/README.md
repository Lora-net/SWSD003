# LR11x0 SDK Bluetooth® Low Energy Beaconing Compatibility sample code

## Description

This example is only available for LR1110 and LR1120.

Technical details regarding Bluetooth Low Energy Beaconing Compatibility commands and usage are explained on the LR11x0 User Manual (available on the [LR1110 product page](https://www.semtech.com/products/wireless-rf/lora-edge/lr1110) and [LR1120 product page](https://www.semtech.com/products/wireless-rf/lora-edge/lr1120)).

## Configuration

Several parameters can be updated in [`../../common/apps_configuration.h`](../../common/apps_configuration.h) header file, refer to [`../../common/README.md`](../../common/README.md) for more details.

Several parameters can be updated in [`main_bluetooth_low_energy_beaconing_compatibility.h`](main_bluetooth_low_energy_beaconing_compatibility.h) header file:

| Constant                                | Comments                                       |
| --------------------------------------- | ---------------------------------------------- |
| `BLUETOOTH_LOW_ENERGY_BEACON_CHANNEL`   | Bluetooth channel used for beacon transmission |
| `BLUETOOTH_LOW_ENERGY_BEACON_PERIOD_MS` | Beacon transmission period [ms]                |
| `BLUETOOTH_LOW_ENERGY_BEACON_TX_POWER`  | Beacon transmission power [dBm]                |

## Beacon mode

The actual implemented beacon mode is Eddystone Beacon.

To use another beacon mode, the user has to modify the Advertizing Channel PDU variable `pdu_buf` within the [source code](./main_bluetooth_low_energy_beaconing_compatibility.c).

## Usage

After building and flashing the code onto the Nucleo board, the LR11x0 chip is expected to transmit a Bluetooth® Low Energy beacon that can be detected with a Bluetooth® Low Energy beacon sniffer device.

This example is exclusively transmitting Bluetooth® Low Energy beacons. It does not detect nor receive Bluetooth® Low Energy beacons.

Note: Semtech’s products are designed to be used in connection with qualified Bluetooth products and applications but are not certified or qualified Bluetooth® products.

### Calibration specific to LR1110

For Bluetooth® Low Energy beaconing compatibility, the LR1110 requires a PLL_TX calibration sequence to be performed prior to the first Bluetooth® Low Energy beaconing-compatible transmission.

Refer to Section 13 in the LR1110 Transceiver User Manual for additional information.

The example here implements this calibration when an LR1110 chip is detected.
