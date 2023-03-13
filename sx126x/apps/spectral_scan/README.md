# SX126x Spectral Scan example

## Description

The application implements Spectral-Scan operation by setting the device in `Rx continuous` mode and regularly reading instantaneous RSSI one frequency channel after the other. For each channel, RSSI value is sampled `NB_SCAN` times by using `GetRssiInst` function. The chip is then switched to next channel to repeat the same process. There is 1 statistic for each frequency channel, which is formed as histogram. On terminal screen, the histogram is displayed as an array of numbers following corresponding frequency value. All those statistics then comprise a heat map of electro-magnetic environment. In every histogram statistic, every number represents the height of the histogram column and the order for the array indicates RSSI values ranging from 0dBm to -128dBm. The length of the array depends on the scale of RSSI level, `RSSI_SCALE`. The X-axis represents background electro-magnetic noise level, while Y-axis represents the possibility of the results locating in the level slot.

Here is an example. Let's say we get a line of "INFO: 915.000 MHz: 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 16 76 8 0 0 0 0 0" in terminal. We can count that the length of this array is 33, so `RSSI_SCALE = (RSSI_TOP_LEVEL-RSSI_BOTTOM_LEVEL)/(33-1) = (0-(-128))/32 = 4dBm`. This would be equivalent to the histogram below.

    0   0   0   0 ... 0   16  76  8   0   0   0   0   0
  ^
  |                            _
  |                           | |
  |                           | |
  |                           | |
  |                           | |
  |                        _  | |
  |                       | | | |  _
  | _____________._._.____| |_| |_| |__________________
  +-------------- ~~~ -------------------------------------->
   /0dBm   /-8dBm ... /-96dBm /-104dBm/-112dBm/-120dBm/-128dBm

The sample code will be used to perform test under both LoRa and FSK modem tests, but there should be no difference if the band-width is the same. Define macro `PACKET_TYPE` to `SX126X_PKT_TYPE_LORA` or `SX126X_PKT_TYPE_GFSK` (in file (`../../common/apps_configuration.h`)) to enable each modem in the test.

## Configuration

Several parameters can be updated in `../../common/apps_configuration.h` header file:

| Constant           | Comments                     | Possible values                             | Default Value           |
| ------------------ | ---------------------------- | ------------------------------------------- | ----------------------- |
| `PACKET_TYPE`      | Set the modem will be used   | Any value from the enum `sx126x_pkt_type_t` | `SX126X_PKT_TYPE_LORA`  |
| `LORA_BANDWIDTH`   | BandWidth of LoRa packets    | Any value from the enum `sx126x_lora_bw_t`  | `SX126X_LORA_BW_125`    |
| `FSK_BANDWIDTH`    | BandWidth for GFSK packets   | Any value from the enum `sx126x_gfsk_bw_t`  | `SX126X_GFSK_BW_117300` |

When compiling with arm-none-eabi-gcc toolchain, all these constant are configurable through command line with the EXTRAFLAGS.
See main [README](../../../README.md).

Several parameters can be updated in `main_spectral_scan.h` header file:

| Constant                | Comments                                        | Default Value  |
| ----------------------- | ----------------------------------------------- | -------------- |
| `FREQ_START_HZ`         | First channel frequency to do the scan          | 915000000      |
| `NB_CHAN`               | Number of channels need to scan                 | 30             |
| `NB_SCAN`               | Number of scan points for each frequency scan   | 100            |
| `PACE_S`                | Number of seconds between 2 scans in the thread | 1              |
| `WIDTH_CHAN_HZ`         | Width between each channel                      | 10000          |
| `RSSI_TOP_LEVEL_DBM`    | Highest RSSI value, default: 0dBm               | 0              |
| `RSSI_BOTTOM_LEVEL_DBM` | Lowest RSSI value, default: -128dBm             | -128           |
| `RSSI_SCALE`            | RSSI scale for spectral scan display            | 4              |
