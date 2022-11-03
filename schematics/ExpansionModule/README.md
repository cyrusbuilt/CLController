# Configuration
The expansion module requires a certain amount of configuration before it can be used.

1) 5V DC power should be applied to JP1. This can be a separate supply or wired in parallel with the main module's supply if using a high enough amperage.
2) The SDA and SCL lines should be wired to JP2 from the main controller.
3) The address needs to be configured using jumpers J3 - J5. The jumpers need to be set to either 1 or 0, they cannot be omitted. See datasheet for MCP23017. Address 32 (all 3 jumpers set to 0) is not allowed as it is reserved by the main controller.
4) By default, I2C termination jumpers (J1 and J2) should both be set to "ENABLE".

## Expansion and Termination
By default, the I2C termination jumpers should be set to "ENABLE" (as mentioned in number for above). This enables the terminators for the SDA and SCL lines for the I2C bus for the last device in the chain. However, if you wish to add another expansion module, you should connect the outgoing SDA and SCL signals to JP3 and set the I2C termination jumpers to "DISABLE".

## Restrictions
The following restrictions apply:
1) No more than 7 expansion modules can be connected to a main controller.
2) The last expansion module in the chain *MUST* have it's termination jumpers set to ENABLE.
3) If running the main controller plus one or more expansion modules from the same power supply, it *MUST* supply enough amperage to satisfy the power requirements of all the modules.
4) Jumper wires connecting I2C lines should be kept as short as possible.