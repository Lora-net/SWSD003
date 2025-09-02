# SMTC Shields

# Purpose
This library contains the software implementation regarding the evaluation kit shields, which contain the radio and the RF components, useful to be called by other parts of the Semtech software.

# Software structure
This chapter briefly describes the software architecture this library.

### SX128X, SX126X and LR11XX families

#### Headers
- File <chip_family>/inc/smtc_shield_<chip_family>_types.h contains information about the types shared by all the shields.
- File <chip_family>/inc/smtc_shield_<chip_family>.h contains the interfaces that will be implemented by every shield. It contains the declaration of the function pointers that will then be rerouted and defined in the specific shield file.
- Each <chip_family>/inc/smtc_shield_<shield_name>.h contains definition of the instantiation, so that the function pointers are correctly assigned, and then the declaration of the functions used by every shield.

#### Modules
- Each <chip_family>/inc/smtc_shield_<shield_name>.c contains the definition of the functions specific to a shield.

#### Modules
- Each <chip_family>/inc/smtc_shield_<shield_name>.c contains the definition of the functions specific to a shield.

# Supported chip families
At the moment, these chip families are supported, you can check on the readmes to know which specific EVKs are supported:
- **LR11XX**: [LR11XX readme](lr11xx/README.md)
- **SX126X**: [SX126X readme](sx126x/README.md)
- **SX128X**: [SX128X readme](sx128x/README.md)
