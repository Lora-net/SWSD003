# LR11XX Scan operations

This document provides description for GNSS and Wi-Fi scanning operations with LR11XX Transceiver product.
It gives high level state machine of these scanning operation.

This document is not a technical description of the LR11XX driver, but it may reference it when appropriate.
It not a detailed description of the internals of the LR11XX.
It does not provide Wi-Fi standard information.

This document describes only the LR11XX Transceiver product, and not the *LR11XX Modem* product.
However, the scanning operations are similar between the two products.

## Important note

The LR11XX is **NOT** capable to locate itself without external assistance.
It only sense its surrounding environment to provide location capable information that are used by a third-party to actually obtain the location of the LR11XX.

## Wi-Fi Passive Scanning operations

The *Wi-Fi Passive Scanning* feature of the LR11XX refers to the capacity of the LR11XX to acquire Wi-Fi signals of its surrounding environment, and extract some location capable information.

There are two Wi-Fi scanning sources of information provided by the LR11XX Transceiver:

- MAC addresses
- Country Codes

Additional information are also available under the form of *metadata*:

- Signal RSSI
- Channel
- Wi-Fi type

There are more metadata available. Refers to the LR11XX datasheet for the complete Wi-Fi Passive Scanning information available.

### Wi-Fi Passive Scanning state machine

The Wi-Fi Passive Scanning operation is simple:

1. Call the Wi-Fi scan API
1. Wait for the Wi-Fi Scan Done IRQ
1. Fetch the results
    1. Get result size
    1. Read results

#### Calling Wi-Fi Passive Scan API

There are two scan APIs available to retrieve location information with Wi-Fi: The *Wi-Fi Scan* and the *Search Country codes*.
These APIs have configurable elements to tune the Passive Scan operation. It allows to tune the energy consumed per scan operation.

##### Search Country codes API

The *Search Country code* allows to get the country codes transmitted by some Wi-Fi frames.
It makes this information directly available by the device.
However, there is no ensurance that the Country Code in the Wi-Fi frame is correct. So a device should not rely solely on this element to establish its location.

The configurable API elements are:

- Channel mask of the channel to scan
- Maximal number of single MAC addresses to extract
- Maximal number of scanning attempts per channel
- Search timeout per attempt
- Flag to indicate channel switch on first timeout

##### Wi-Fi Scan API

The *Wi-Fi Scan* API configure and start the extraction of Wi-Fi MAC addresses in the surrounding environment of the LR11XX.
The configuration elements are:

- Wi-Fi signal type of frame to search (Wi-Fi B or Wi-Fi G)
- Channel mask of the channel to scan
- Scan mode (Wi-Fi frame selection) (Scan for Wi-Fi beacon only or Wi-Fi beacon and data paquets)
- Maximal number of single MAC addresses to extract
- Maximal number of scanning attempts per channel
- Search timeout per attempt
- Flag to indicate channel switch on first timeout

#### Waiting for Wi-Fi Scan Done IRQ

Once the LR11XX has started its Wi-Fi scanning operation, it does not expect action from host microcontroller.
Additionnaly, it is not possible to use other LR11XX features during the Scanning operation.
The host microcontroller must waits for the Scanning operation to be terminated before doing any operation on the LR11XX.

The LR11XX ensures that an IRQ will raise afer calling the Wi-Fi Scan API. Even if there is no results or if the configuration did not allow to start a single attempt (for instance if maximal result was set to 0).

When an IRQ has raised, two elements are to be fetched : the IRQ mask and the status of the operation.
The IRQ mask ensures that the IRQ raised by the LR11XX is indeed a Wi-Fi Scan done interrupt. If it is not the case, the results must not be fetched as it can be the consequence of a bug (unterminated previous operation for instance).

If the IRQ raised is indeed a Wi-Fi Scan done interrupt, then the status is used to indicate if the IRQ has been raised because the command succesfully terminated, or if the command failed to start. The Scan command may fail to start in case of wrong value provided to configuration parameters for instance.

If the status does not indicate the Scan command successful termination, then the result may not be fetch, as it can be the result of a previous scan, or the result may be corrupted.

#### Fetching Wi-Fi Scanning operation results

The result of a Scanning operation can be fetched only if the Wi-Fi Scan done IRQ raised with status *Command success*.

The LR11XX exposes different APIs to fetch Wi-Fi scanning results depending on the Wi-Fi scan API issued at call step.
In both cases, the strategy is the same: once the Wi-Fi Scan done IRQ has raised, first fetch the result size, the fetch the actual result data.
The fetching of result size **MUST** be issued before the fetching of the results.

##### Fetching Country Code results

The API to get the result size of Country Code results is *Get number country code results*. It returns the number of Country code scanned from single MAC address.
Then the API to fetch the result is *Read Country Code*.

##### Fetching Wi-Fi Scanning results

As for the Country Code, the API *Get number scan results* must be issued first. It returns the number of single MAC address scanned.
Then the API *Wi-Fi Get Results* can then be issued to read the actual Wi-Fi Scanning results.
This API takes an additional parameter which is the result structure type to return.
Refer to the LR11XX datasheet to have the possible structure types.

The timing of the different internal steps are also available to host microcontroller by issuing the *Get timings* API. It is useful to compute the energy consummed by the Wi-Fi scanning operation.

## GNSS Scanning operation

The GNSS Scanning operation refers to the capability of the LR11XX to extract information from GNSS signals.
This information is then to be used by a third-party to solve the location of the LR11XX.

The GNSS Scanning operation output is a *NAV message*. A NAV message is a byte array that contains information allowing the solving of a device location from GNSS signal characteristics.

For satellite signal acquisition, there are two possible Scanning operations:

- Autonomous GNSS scanning
- Assisted GNSS scanning

The *Autonomous GNSS scanning* search for strong GNSS signals coming from any satellites. *Assisted GNSS scanning* takes advantage of user-provided *Assitance information* to quikly search for specific GNSS signals. For instance, the rough location (150 km accurate) of the device and up-to-date almanacs helps the LR11XX to quiclky search for GNSS signals of the satellites it is supposed to see.

On top of *Assisted* and *Autonomous* operations, there are two scanning modes available:

- Single capture mode
- Double capture mode

With Single capture mode, the LR11XX execute a single GNSS signal acquisition. With Double capture mode, it executes two consecutive acquisition.
In the later case, the duration between the two executions must be precisely of 30 seconds. It is up to the host microcontroler to start the execution of the second scan operation. This leads to two different state machines depending on this parameter.

The LR11XX is capable to scan several GNSS constellations. However, it is not capable to acquire signal of several constellation *at the same time*. Therefore, when enabling several constellations for a single GNSS scanning operation, the LR11XX will actually execute two signal acquisition in a row.
On the opposite of double capture, the additionnal signal acquisition required for multiple constellation is driven by the LR11XX alone, without requiring activity from the host microcontoller.

### State machines

#### Single capture mode

In single capture mode, the state machine is closed to the Wi-Fi one:

1. Configuration of the GNSS scanning operation
1. Start execution of the GNSS scanning operation (*Autonomous* or *Assisted* Scanning operation)
1. Wait for interrupt
1. Fetch NAV message

#### Double capture mode

1. Configuration of the GNSS scanning operation
1. Start execution of the GNSS scanning operation (*Autonomous* or *Assisted* Scanning operation)
1. Wait for interrupt
1. Wait for 30 seconds
1. Start the execution of the second scanning operation
1. Wait for interrupt
1. Fetch NAV message

#### Configuration of the GNSS scanning operation

##### Configuration common to Autonomous and Assisted scanning opertation

There are two APIs to call as configuration step prior start the execution of the scanning operation:

- *Set constellation mask* to indicate which GNSS constellations are to be scanned
- *Set scan mode* to select single or double scanning operation

The *Set scan mode* API also return the duration the host microcontroller must respect between the two consecutive signal acquisition in the case of double scan (or `0` in the case of a single scan).

##### Configuration specific to Assited scanning operation

To execute an *Assisted* scanning operation, it is required to configure the rough location prior to start the scanning operation. This is achieved by calling the *Set assistance position*.

The satellite almanac must be kept up-to-date for the LR11XX to find satellite signals during Assisted scanning operation. The update of Almanac is done by issuing the *Almanac Full Update* API.

An almanac is valid for about 3 months. After that period of time, the LR11XX cannot execute an assisted scanning operation. However, when using almanac older than 1 month, the LR11XX intentionally extended its GNSS signal search to overcome the inaccuracy of almanacs.
This increase the power consumed for Assisted scanning operation.

The date of generation of the almanac is available by reading the almanac content from the LR11XX. This is done by first using the *Get almanac address and size* API that returns the base address and size of the almanac memory location.
The almanac memory is splitted into 22 bytes long chunk, each one corresponding to the almanac of a satellite. The almanac age is the 16 bits area of each chunk located at byte 1 and 2 (starting to count at 0).
The almanac age is expressed in days elapsed since last GPS rollover.

#### Execution of the GNSS scanning operation

This step is executed in both Single capture and Double capture mode. It is not the one used for the execution of the second capture in the case of Double capture mode.

The APIs for *Autonmous* and *Assitance* scanning operations are different.

##### Execute Autonomous scanning operation

The API *Scan Autonomous* takes the following parameters:

- GPS date of the scan
- Input parameter bit mask
- Number of satellites to search for

In Autonomous scanning, the date is not actually used by the LR11XX. However, it is inserted in the generated NAV message to be used later on by the solver.

The Input parameter bit mask allows to select the information contained in the NAV message, allowing to reduce its size. Refer to LR11XX datasheet for details.

##### Execute Assisted scanning operation

The API *Scan Assisted* takes the following parameters:

- GPS date of the scan
- Effort mode
- Input parameter bit mask
- Number of satellites to search for

The date is actuallly actually used in conjonction with the almanac to compute the satellite signals the LR11XX is supposed to find.
This allows the LR11XX to avoid consumming energy to search for all satellite signals.

The effort mode is a flag that avoid additionnal steps to be performed if all required satellites are already found. Refer to LR11XX datasheet for more details.

#### Wait for interrupt

This step is common to Single scanning ans Double scanning operations. Note that it appears twice in the Double scanning operation.

It is similar to the Wi-Fi wait for interrupt step.
Before continuing the GNSS scanning operation, the host microcontroller must ensure that the IRQ generated is indded a *GNSS scan done* interrupt, and that the status is *Command Ok*.
If the status is not *Command Ok*, then the GNSS Scanning state machine must stop here as the results availalbe can be corrupted, or the one from a previous scanning operation.

#### Wait for 30 seconds

This step is **ONLY** for Double scanning operation.

In the case of Double scanning operation, if the interrupt is a *GNSS scan done* interrupt, and the status is *Command Ok*, then the host must waits for 30 seconds before starting the second GNSS scan operation.

These 30 seconds are defined as the duration between the start of the first scan, and the start of the second scan.
It is up to the host microcontroller to ensure this duration is respected.

#### Start execution of the second sniff

This step is **ONLY** for Double scanning operation.

To start the second scanning operation of a Double scan, the host microcontroller must calls the API *Scan continuous*. This API takes no parameters.

It will generate an interrupt *GNSS scan done* when it terminates, that must be handled as described in *Wait for interrupt* section.

#### Fetch NAV message

This steps occurs for both Single and Double scanning operations.

To fetch the NAV message, the host microcontroller must first issues the API *Get result size*.
It will returns the number of bytes of the NAV message to read.

Then, the microcontroller must uses the *Read result* API to read the NAV message.

A NAV message starting by byte 0x00 indicates it is directed to the host microcontroller. A NAV message starting by byte 0x01 indicates it is for the solver. Refers to the LR11XX datasheet for all values and their meaning.

Additionnal information are available at this step.
The IDs and the SNR of the satellites are available by calling the *Get number of detected satellites* followed by the *Get detected satellites* APIs.
The API *Get timings* returns the duration of internal steps in the GNSS signal acquisition and analysis that are useful to compute the energy consumed by the LR11XX for the scanning operation.

## Ressources

- LR11XX Datasheet
- In-code driver documentation
