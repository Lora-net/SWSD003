# LR11XX SDK TX LR-FHSS sample code

## Description

The application will automatically configure the device to transmit packets in LR-FHSS.

## Configuration

Frequency can be updated in [`../../common/apps_configuration.h`](../../common/apps_configuration.h) header file, refer to [`../../common/README.md`](../../common/README.md) for more details.

Several parameters can be updated in [`main_tx_lr_fhss.h`](main_tx_lr_fhss.h) header file:

| Constant                  | Comments                                  |
| ------------------------- | ----------------------------------------- |
| `TX_TO_TX_DELAY_IN_MS`    | Time delay between 2 transmitting packets |
| `LR_FHSS_BANDWIDTH`       | Bandwidth                                 |
| `LR_FHSS_CODING_RATE`     | coding rate                               |
| `LR_FHSS_ENABLE_HOPPING`  | Enable or no the frequency hoping         |
| `LR_FHSS_GRID`            | grid                                      |
| `LR_FHSS_HEADER_COUNT`    | Number of header blocks                   |
| `LR_FHSS_MODULATION_TYPE` | Modulation type                           |
