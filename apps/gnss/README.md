# LR11xx GNSS scan example

## Description

The application executes GNSS scans in a loop. It prints results on the serial line on every scan termination.

## Configuration

Several parameters can be updated in [`main_gnss.h`](main_gnss.h) header file:

| Constant                             | Comments                                                |
| ------------------------------------ | ------------------------------------------------------- |
| `GNSS_ASSISTANCE_POSITION_LATITUDE`  | Assistance position latitude (GNSS assisted scan only)  |
| `GNSS_ASSISTANCE_POSITION_LONGITUDE` | Assistance position longitude (GNSS assisted scan only) |
| `GNSS_SCAN_MODE`                     | Scan mode                                               |
| `GNSS_EFFORT_MODE`                   | Effort mode                                             |
| `GNSS_INPUT_PARAMETERS`              | GNSS input parameters                                   |
| `GNSS_MAX_SV`                        | Maximum number of SV to search per scan                 |
| `GNSS_MAX_SCANS`                     | Maximum number of scan to execute in a row              |

The selection of GNSS scan autonomous or GNSS scan assisted is achieved by selecting the toolchain from the correct folder, as follows:

| GNSS scan selection | Makefile                                                                 | Keil                                                                                                                       |
| ------------------- | ------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------- |
| GNSS autonomous     | [`gnss_autonomous/makefile/Makefile`](gnss_autonomous/makefile/Makefile) | [`gnss_autonomous/MDK-ARM/lr11xx_sdk_gnss_autonomous.uvprojx`](gnss_autonomous/MDK-ARM/lr11xx_sdk_gnss_autonomous.uvprojx) |
| GNSS assisted       | [`gnss_assisted/makefile/Makefile`](gnss_assisted/makefile/Makefile)     | [`gnss_assisted/MDK-ARM/lr11xx_sdk_gnss_assisted.uvprojx`](gnss_assisted/MDK-ARM/lr11xx_sdk_gnss_assisted.uvprojx)         |

## GNSS assisted example details

### Almanacs

The GNSS assisted example needs the almanac to be up-to-date. If the almanacs are too old, the LR11xx chip is likely to detect less Satellite Vehicles. And if the almanacs are older than three months the LR11xx chip will not start a scanning operation and will return an error result `00 08` indicating the scan was not attempt because the almanacs are too old.

To update the almanacs consider using the [LR1110 EVK Demo Application](https://github.com/Lora-net/lr1110_evk_demo_app) and the python program `AlmanacUpdate` as described [here](https://github.com/Lora-net/lr1110_evk_demo_app#almanacupdate-usage). This process can be followed for both LR1110 and LR1120.

### Assistance information

To execute GNSS assisted scans, the LR11xx chips needs the following additional information:

- GPS time
- assistance position

The assistance position is an estimation of the device location (with 150 km accuracy). It is expressed as a coordinate in decimal degree, latitude is in the macro `GNSS_ASSISTANCE_POSITION_LATITUDE` and longitude in `GNSS_ASSISTANCE_POSITION_LONGITUDE`.

The GPS time however needs to be given to the device while it is running. This is achieved by using the python script `apps/gnss/gnss_assisted/host.py`.
This script simply get the time from the machine it is running on, and convert it to GPS time. This GPS time is then sent to the the device through the serial port.

The script is to be used as follows:

```bash
$ python host.py <COM_PORT>
```

Where `<COM_PORT>` is the serial COM port where the nucleo board is connected.

Note that this scripts just send the GPS time on serial port when it starts, then it prints the log coming from the device. So the device must be already started before executing the python script.
