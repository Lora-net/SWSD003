# LR11XX Sigfox PHY sample code

## Description

This sample code illustrates the sending of Sigfox-compliant uplinks.

### Payload setting

There is currently one pre-defined physical payload (corresponding to "0x01" application payload).

To send another payload, one can update `sample0` array and `SIGFOX_PAYLOAD_LENGTH` macro in [`main_sigfox.c`](main_sigfox.c).

## Configuration

Several parameters can be updated in [`../../common/apps_configuration.h`](../../common/apps_configuration.h) header file, refer to [`../../common/README.md`](../../common/README.md) for more details.

Several parameters can be updated in [`main_sigfox.h`](main_sigfox.h) header file:

| Constant               | Comments                                  |
| ---------------------- | ----------------------------------------- |
| `TX_TO_TX_DELAY_IN_MS` | Time delay between 2 transmitting packets |
