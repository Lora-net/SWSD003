# LR11xx Wi-Fi scan example

## Description

The application executes Wi-Fi Scans in a loop.
The Wi-Fi scans can be the following:
- Wi-Fi passive scan
- Wi-Fi passive scan time limit
- Wi-Fi passive scan country code
- Wi-Fi passive scan country code time limit

It prints results on the serial line on every scan termination.
Different result format are possible depending on the Wi-Fi scan type selected and configuration.

## Configuration

Several parameters can be updated in [`main_wifi.h`](main_wifi.h) header file. Some are common to all Wi-Fi scan types, and some depends on it:

### Parameters common to all Wi-Fi scan types

| Constant                  | Comments                                                     |
| ------------------------- | ------------------------------------------------------------ |
| `WIFI_DEMO_TO_RUN`        | Wi-Fi scan type selection. Must be from `wifi_demo_to_run_t` |
| `WIFI_CHANNEL_MASK`       | Wi-Fi channels to scan                                       |
| `WIFI_MAX_RESULTS`        | Maximum number of results to scan for                        |
| `WIFI_MAX_NUMBER_OF_SCAN` | Maximum number of scan to execute. Infinit if set to `0`     |

### Parameters for Wi-Fi passive scan and Wi-Fi passive scan time limit

| Constant             | Comments                                                                                             |
| -------------------- | ---------------------------------------------------------------------------------------------------- |
| `WIFI_SIGNAL_TYPE`   | Wi-Fi signal type to scan for                                                                        |
| `WIFI_SCAN_MODE`     | Wi-Fi scan mode to use                                                                               |
| `WIFI_RESULT_FORMAT` | Wi-Fi result format to use. Possible values depend on `WIFI_SCAN_MODE` (see doxygen command in-code) |

### Parameters for Wi-Fi passive scan and Wi-Fi passive scan country code

| Constant                   | Comments                                                   |
| -------------------------- | ---------------------------------------------------------- |
| `WIFI_NB_SCAN_PER_CHANNEL` | Number of scan per channel to execute                      |
| `WIFI_TIMEOUT_PER_SCAN`    | Maximum duration of the scan process (ms)                  |
| `WIFI_ABORT_ON_TIMEOUT`    | Abort scan on one channel on detection of preamble timeout |

### Parameters for Wi-Fi passive scan time limit and Wi-Fi passive scan country code time limit

| Constant                   | Comments                                     |
| -------------------------- | -------------------------------------------- |
| `WIFI_TIMEOUT_PER_CHANNEL` | Maximum scan duration spent per channel (ms) |
| `WIFI_TIMEOUT_PER_SCAN`    | Maximum duration of the scan process (ms)    |
