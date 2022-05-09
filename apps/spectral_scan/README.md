# LR11xx Spectral Scan example

## Description

The application implements Spectral-Scan operation by setting the device in `Rx continuous` mode and regularly reading instantaneous RSSI one frequency channel after the other. For each channel, RSSI value is sampled `NB_SCAN` times by using `GetRssiInst` function. The chip is then switched to next channel to repeat the same process. There is 1 statistic for each frequency channel, which is formed as histogram. On terminal screen, the histogram is displayed as an array of numbers following corresponding frequency value. All those statistics then comprise a heat map of electro-magnetic environment. In every histogram statistic, every number represents the height of the histogram column and the order for the array indicates RSSI values ranging from 0dBm to -128dBm. The length of the array depends on the scale of RSSI level, `RSSI_SCALE`. The X-axis represents background electro-magnetic noise level, while Y-axis represents the possibility of the results locating in the level slot.

Here is an example. Let's say we get a line of "INFO: 2400.000 MHz: 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 16 76 8 0 0 0 0 0" in terminal. We can count that the length of this array is 33, so `RSSI_SCALE = (RSSI_TOP_LEVEL-RSSI_BOTTOM_LEVEL)/(33-1) = (0-(-128))/32 = 4dBm`. This would be equivalent to the histogram below.

```
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
```

The sample code will be used to perform test under both LoRa and FSK modem tests, but there should be no difference if the band-width is the same. Define macro `PACKET_TYPE` to `LR1110_RADIO_PKT_TYPE_LORA` or `LR1110_RADIO_PKT_TYPE_GFSK` (in file [`../common/apps_configuration.h`](../common/apps_configuration.h)) to enable each modem in the test.

## Configuration

Several parameters can be updated in [`../common/apps_configuration.h`](../common/apps_configuration.h) header file:

| Constant           | Comments                    |
| ------------------ | ----------------------------|
| `PACKET_TYPE`      | Set the modem will be used  |
| `LORA_BANDWIDTH`   | BandWidth of LoRa packets   |
| `FSK_BANDWIDTH`    | BandWidth for GFSK packets  |

Several parameters can be updated in [`main_spectral_scan.h`](main_spectral_scan.h) header file:

| Constant                | Comments                                        |
| ----------------------- | ----------------------------------------------- |
| `FREQ_START_HZ`         | First channel frequency to do the scan          |
| `NB_CHAN`               | Number of channels need to scan                 |
| `NB_SCAN`               | Number of scan points for each frequency scan   |
| `PACE_S`                | Number of seconds between 2 scans in the thread |
| `WIDTH_CHAN_HZ`         | Width between each channel                      |
| `RSSI_TOP_LEVEL_DBM`    | Highest RSSI value, default: 0dBm               |
| `RSSI_BOTTOM_LEVEL_DBM` | Lowest RSSI value, default: -128dBm             |
| `RSSI_SCALE`            | RSSI scale for spectral scan display            |
