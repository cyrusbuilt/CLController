# CLController Schematics

All files were produced using [EasyEDA](https://easyeda.com/). The following is a list of files and what they are:

- CLController/COM_CLController.csv - The bill of materials in CSV format.
- CLController/PCB_CLController.pdf - A PDF version of the PCB view.
- CLController/SCH_CLController.pdf - A PDF version of the schematic view.
- CLController/PCB_CLController.json - The original EasyEDA CAD file of the schematic.
- CLController/SCH_CLController.json - The original EasyEDA CAD file of the PCB.
- CLController/gerber - Subdirectory containing the exported Gerber files useful for importing into other CAD programs or for sending to fab houses that do not support EasyEDA files.
- ExpansionModule/* - All of the same things as above, but for the I/O expansion module.
- RelayModule/* - All of the same things as above, but for the relay module.

## NOTES

The MCU used in Rev 1.x of CLController is an Adafruit Huzzah ESP8266.  Future versions of CLController may switch to using an ESP32-based MCU because it is dual-core. Currently, (as is the case with most Arduino-compatible SoCs) there is only one core and only one main loop.  All processing happens on this main loop. While the playsheet processing loop is running, the other subsystems have to wait (task manager, MQTT message processing, OTA request processing, etc) until a sequence delay finishes. All subsystem operations are in their own loop so they can be called from the playsheet sequence processing loop as well as from the main loop() method. This way we can still process everything else, ableit delayed. A better solution would be to use a multi-core MCU like the ESP32 so that the sequence processing can execute on a separate core and not interfere with processing all the other subsystems.