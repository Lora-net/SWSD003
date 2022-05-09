# LR11xx Spectrum Display example

## Description

The application implements Spectrum-Display operation by setting the device in `Rx continuous` mode and regularly reading instantaneous RSSI one frequency channel after the other. The spectrum covering all the channels being scanned will be plot on terminal screen. From the start channel frequency, defined by `FREQ_START_HZ`, 1 sample point of RSSI level will be fetched for every frequency channel by using `GetRssiInst` function. After collection of all frequency channel RSSI values, a spectrum curve is drawn on terminal screen. This curve will be refreshed every `PACE_S` second(s) by repeating the processes done before.

Here is an example of the curve will be displayed on terminal screen. The bottom of the curve represents noise floor around the antenna in frequency band being scanned. The spikes represent which frequency channels have RF activity happening. The frequency frame displayed below x-axis is the frequency band being scanned. Y-axis indicates for RSSI level.

```
    ^
   0|
  -4|
  -8|
 -12|
 -16|
    .
    .
    .
 -76|
 -80|                          _
 -84|                    _    | |
 -88|                  _| |   | |
 -92|_                |   |   | |              _
 -96| |               |   |   | |             | |              _
-100| |               |   |   | |             | |             |
-104| |            _  |   |_._| |_     _     _| |             |
-108| |_._._._._._| |_|           |_._| |_._|   |_._._._._._._|
-112|
-116|
-120|
-124|
-128|
/dBmx------------------------------------------------------------>
     2400 --> 2406 MHz
```

The plotting function is supported by `VT100` control code. So, to run this demo normally, a terminal like `Tera Term` supporting `VT100` control code is necessary.

The sample code will be used to perform test under both LoRa and FSK modem tests, but there should be no difference if the band-width is the same. Define macro `PACKET_TYPE` to `LR1110_RADIO_PKT_TYPE_LORA` or `LR1110_RADIO_PKT_TYPE_GFSK` (in file [`../common/apps_configuration.h`](../common/apps_configuration.h)) to enable each modem in the test.


## Configuration

Several parameters can be updated in [`../common/apps_configuration.h`](../common/apps_configuration.h) header file:

| Constant           | Comments                    |
| ------------------ | ----------------------------|
| `PACKET_TYPE`      | Set the modem will be used  |
| `LORA_BANDWIDTH`   | BandWidth of LoRa packets   |
| `FSK_BANDWIDTH`    | BandWidth for GFSK packets  |

Several parameters can be updated in [`main_spectrum_display.h`](main_spectrum_display.h) header file:

| Constant                 | Comments                                        |
| ------------------------ | ----------------------------------------------- |
| `FREQ_START_HZ`          | First channel frequency to do the scan          |
| `NB_CHAN`                | Number of channels need to scan                 |
| `PACE_S`                 | Number of seconds between 2 scans in the thread |
| `WIDTH_CHAN_HZ`          | Width between each channel                      |
| `RSSI_TOP_LEVEL_DBM`     | Highest RSSI value, default: 0dBm               |
| `RSSI_BOTTOM_LEVEL_DBM`  | Lowest RSSI value, default: -128dBm             |
| `RSSI_SCALE`             | RSSI scale for Spectrum-Display display         |
