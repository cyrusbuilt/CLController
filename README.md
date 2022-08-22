# CLController

ESP8266-based Christmas Lights Controller (Arduino)

![Build Status](https://github.com/cyrusbuilt/CLController/actions/workflows/ci.yml/badge.svg)

The code in this repository represents the firmware for the CLController main module, as well as some components for integrating with [OpenHAB3](https://openhab.org). CLController is a [PlatformIO](https://platformio.org) project and is meant to essentially a multi-module outlet controller, with the idea being that you can control multiple string of traditional Christmas light strings (this is not a pixel controller). Given the hardware design however, it is possible to use this device for multiple applications.  I can be monitored and controlled via [MQTT](https://mqtt.org).  Since there is an [ESP8266](https://www.adafruit.com/product/2471) at the heart of it, that means it is WiFi-enabled and is capable of OTA (Over-the-Air) firmware and configuration updates.

## Theory of Operation

CLController consists of one or more combinations of the following 3 hardware components:
- CLController Main Board
- CLController Relay Module
- CLController I/O Expansion Module

The bare-minimum configuration would be the CLController Main Board and (1) Relay Module.  Each relay module allows control of up to 8 circuits at 10A (MAX) per cicuit. The main control board allows up to (2) Relay Modules to be connected to it providing a total of 16 switched circuits.  You can expand this further by connecting an I/O expansion module (up to 7 max - providing a total of 128 possible relays max).  Each expansion module allows you to daisy chain another expansion module and also supports 2 more relay modules per expansion module. The expansion modules connect using the 2-Wire (I2C) bus and are address-configurable (the main control module is hard-wired for address 0x20).  Each expansion module is auto-detected and intialized during boot.

CLController uses a concept of "Playsheets" which is basically a JSON file containing a collection of sequences that fire the relays in the appropriate order at the appropriate time.  Each "sequence" contains a list of "states" followed by a delay in milliseconds.  Each "state" contains the relay module index, light string index (relay number), and state ("on/off").

The following is an example of the playsheet schema:

```json
{
    "name": "Example",
    "seq": [
        {
            "states": [
                {
                    "modIdx": 0,
                    "lsIdx": 1,
                    "lsState": "on"
                },
                ...
            ],
            "delayMs": 600
        },
        ...
    ]
}
```

- name = The play sheet name.
- seq = An array of sequences.
- states = An array of states.
- modIdx = Relay Module Index. 0 is the first relay module (port A on the main board).
- lsIdx = Light String Index. This is the index (address) of the relay to control (1 - 8).
- lsState = The state to set "on" or "off".
- delayMs = The delay in milliseconds before transitioning to the next state.

## IMPORTANT NOTE:

The size of the playsheet you can load will be limited to the amount of RAM available at runtime. If your playsheet (sequence file) is too large, it will fail to load when the controller boots and will report this condition to the serial console. In a future firmware release, the controller will also indicate this as it's status so it can be seen in the status payload published to MQTT and/or indicated using a flashing "RUN" LED or something like that.

## Architecture

CLController currently uses an ESP8266 MCU at it's heart. Unfortunately, the ESP8266 does not have a large number of GPIO pins. So to control the number of relays we need to, the main board (and the I/O expansion board) includes an MCP23017 2 port (8 GPIOs per port) I2C I/O expander paired with (2) ULN2803 Darlington arrays (each provides 8 outputs). The ULN2803s act as relay drivers. Thus allowing us to control up to 16 relays per expander. You can have a maximum of (8) MCP23017 chips on the I2C bus at a time, allowing for a grand total of 128 possible relays per CLController.  You can, of course, set up multiple CLController boards on your network allowing for even more control, but currently, the CLController boards do not communicate with each other and have no way of sharing playsheets. That being said.... if you need to control more than 128 strings of lights on one controller... you have a very impressive setup and an even more impressive utility bill. It's possible that you could break up the sequences across multiple playsheets and send each sheet to each CLController involved OTA, then send the run command to all the controllers over MQTT.  There might be timing issues depending on how long it takes each controller to recieve the command and begin executing though, but something to consider.

## Configuration

The config.h file contains default configuration directives. These options are the hardcoded default values that the system will initially boot with. However, if there is a config.json file present in the root of the SPIFFS file system, then it will override these defaults. Here we'll discuss each option:

- ENABLE_OTA: If you do not wish to support OTA updates, just comment this define.
- ENABLE_MDNS: If you do not wish to support [MDNS](https://tttapa.github.io/ESP8266/Chap08%20-%20mDNS.html), comment this define.
- ENABLE_TLS: If you wish to use TLS support (MQTT-over-SSL), uncomment this define.
- CONFIG_FILE_PATH: This is the path to the configuration file. The default is /config.json. The values in this file will override these defaults.
- DEFAULT_SSID: Change this line to reflect the SSID of your WiFi network.
- DEFAULT_PASSWORD: Change this line to reflect the password to your WiFi network.
- CLOCK_TIMEZONE: The GMT offset for your timezone (EST [America/New York] is -4 when observing DST. It's -5 when not.)
- SERIAL_BAUD: While it is not recommended, you can change the BAUD rate of the serial port here.
- CHECK_WIFI_INTERVAL: The interval (in milliseconds) to check to make sure the device is still connected to WiFi, and if not attempt to reconnect. Default is 30 seconds.
- CHECK_SENSORS_INTERVAL: The interval (in millisceonds) to check the water depth.
- CLOCK_SYNC_INTERVAL: The interval (in milliseconds) to try to resync the clock with NTP. Default is 1 hour.
- DEVICE_NAME: This essentially serves as the host name of the device on the network.
- CHECK_MQTT_INTERVAL: The interval (in milliseconds) to check connectivity to the MQTT broker. If the connection is lost, a reconnect will occur. The default value is 35 seconds.
- MQTT_TOPIC_STATUS: The MQTT status topic to publish device status messages to. Default is 'cysump/status'.
- MQTT_TOPIC_CONTROL: The MQTT control topic to subscribe to for control messages. Default is 'cysump/control'.
- MQTT_BROKER: The hostname or IP of the MQTT broker.
- MQTT_PORT: The port on the MQTT broker to connect to. The default is 8883 (default port for MQTT over TLS).
- OTA_HOST_PORT: Defines the port to listen for OTA updates on. This option is ignored if ENABLE_OTA is disabled.
- OTA_PASSWORD: The password used to authenticate with the OTA server. This option is ignored if ENABLE_OTA is disabled.
- ip: The default IP address. By default, this devices boots with a static IP configuration. The default IP is 192.168.0.202. You can change that here if you wish.
- gw: The default gateway address. The current default is 192.168.0.1. You can change that here if you wish.
- sm: The subnet mask. By default, it is 255.255.255.0, but you can change that here if need be.

To override the default configuration options, you need to upload a filesystem image containing a file named 'config.json' in the root of the SPIFFS filesystem. The file should like something like this:

```json
{
    "hostname": "CLCONTROLLER",
    "useDHCP": false,
    "ip": "192.168.0.202",
    "gateway": "192.168.0.1",
    "subnetMask": "255.255.255.0",
    "dnsServer": "192.168.0.1",
    "wifiSSID": "your_ssid_here",
    "wifiPassword": "your_password_here",
    "webserverPort": 80,
    "otaPort": 8266,
    "otaPassword": "your_ota_password_here",
    "mqttBroker": "your_mqtt_broker_here",
    "mqttPort": 8883,
    "mqttControlChannel": "clcontroller/control",
    "mqttStatusChannel": "clcontroller/status",
    "mqttUsername": "your_mqtt_username",
    "mqttPassword": "your_mqtt_password"
}
```

This configuration file is pretty self explanatory and one is included in the source. The file *MUST* be located in the "data" directory located in the root of the project in order to be picked up by the flash uploader (either via Serial or OTA). Each of the options in the configuration file are self-explanatory and match up with the hard-coded default settings mentioned above. If this file is not present when the firmware boots, a new file will be created and populated with the hardcoded defaults. These defaults can then be changed via the fail-safe menu and saved. You'll notice a couple of additional options in the config.json file not present in config.h. They are as follows:

- mqttUsername: If you have enabled password authentication on your MQTT broker, provide the username here.
- mqttPassword: If you have enabled password authentication on your MQTT broker, provide the password for the username above. If *both* the username and password are provided, then CLController will attempt to connect to the broker with the provided credentials; Otherwise, it will attempt to connect without credentials.
- mqttControlChannel: The MQTT control topic to subscribe to in order to receive device commands.
- mqttStatusChannel: The MQTT status topic to publish device status to.

## Getting Started

After you've configured everything the way you need it (as discussed above), build the firmware by either clicking "Build" in the PlatformIO tasks menu, or by opening a terminal to the project directory and typing:

```bash
> platformio run
```

NOTE: The first time you flash the Huzzah, you need to do so over serial (since the OTA code isn't there yet), but subsequent uploads can be done via OTA if configured properly.

The next thing to do is connect the Huzzah to your computer using an FTDI cable like [this one](https://www.adafruit.com/product/70?gclid=EAIaIQobChMIm7-50ZiZ5AIVlIvICh284QPxEAQYBCABEgJkcPD_BwE) and then configure the port in platformio.ini like so:

```ini
[env:huzzah]
monitor_speed = 115200
monitor_port = /dev/cu.usbserial-AL05HSL2  ; Change this to match your port if necessary
```

With the above mentioned FTDI cable attached to my MacBook, the port appears as it does in the config file above (usually PlatformIO is pretty good about auto-detecting the port for you).

Now all you have to do is flash the firmware onto the Huzzah. You can do this by first pressing and holding the "GPIO" button and then press the "reset" button and let go of both on the Huzzah to put it into flash mode (this is not necessary when you flash via OTA), then click the "Upload and Monitor" task in the PlatformIO menu or by typing the following into the terminal:

```bash
> platformio run --target upload
> platformio device monitor --baud 115200 --port /dev/cu.usbserial-AL05HSL2
```

Once complete, press the "reset" button on the Huzzah. You should see the device boot up in the serial console. Now put the device back into flash mode.  Configure the playsheet.json and config.json files as needed and click the "Upload File System Image" task in the PlatformIO menu. When finished, press the "reset" button again and the device will reboot and come back up with your new configuration settings.

## OTA Updates

If you wish to be able to upload firmware updates Over-the-Air, then besides leaving the ENABLE_OTA option uncommented, you will also need to uncomment all the upload_* lines in platformio.ini, and change the line 'upload_port = ' line to reflect the IP of the device and the line '--auth=' to reflect whatever OTA_PASSWORD is set to. Then when you click "upload" from the PlatformIO tasks (or if you execute 'platformio run --target upload' from the command line) it should upload directly over WiFi and once completed, the device will automatically flash itself and reboot with the new version. If you wish to upload a new configuration file, you can also do this via OTA. Assuming the above-mentioned settings are configured, you can then click "Upload File System Image" from the PlatformIO project tasks.

## Serial Console

If the device ever fails to connect to WiFi or if you press the 'I' key on your keyboard while in the serial console, normal operation of the device is suspended and the device will fall into a 'fail-safe' mode and present the user with a command menu, which can be use to re-configure the device, reboot, manually turn lights on and off, etc.

## Dependencies

The firmware dependencies will be installed automatically at compile-time. However, if you wish to install dependencies prior to compiling (for intellisense or to clear warnings/errors in the VSCode editor) you can run the following command:

```bash
> platformio lib install
```

## Actually controlling the Christmas lights

This is the dangerous part.  The generaly idea is to use the relays to switch strings of lights on and off.  There are a number of ways to do this:

1) Splice into the outlet wiring of an existing outlet/power strip/etc.
2) Cut one or more extension cords and wire into the relays.
3) Cut one of the power wires on the light strings themselves and wire into the relays.
4) Use a bussed electrical circuit that uses the relays to switch individual circuits (my preferred choice).

DISCLAIMER: You are dealing with HIGH VOLTAGE here.  Mains wiring is absolutely lethal and I AM NOT LIABLE for any injuries or death or damage to property as a result of any mistakes you or anyone else make implementing this controller in any way, shape, or form. If you don't know what you are doing, DON'T DO IT. Ask a qualified engineer or electrician to do this part for you. That being said, the way I did it was to mount all the electronics in an empty electrical box using stand-offs. Then I mounted 3 high-voltage electrical bus bars like those found in a breaker box. I then bought a heavy duty extension cord that can handle at least 15 Amps @ 120 Volts AC and cut the end off. I wired the 3 leads into the 3 different bus bars (which were mounted with sufficient distance from each other). Then I bought 16 lower-grade cheap extension cords and cut the plugs off and wired the ground and neutral wires into the matching bus bars. I wired the hot wire from each into the numbered terminals on the relays and then labeled each extension cord. I then cut pieces of heavy guage wire and used those to wire the COM on each relay to the Hot bus bar.  That way I can plug the box into a 15A outlet and then plug each string into the separate extension cord coming out of the box. (If you do this, you need to verify the outlet you are plugging into provides at least 15A and there is no other load on that circuit that would lead to an overload... It might even be worth adding a fuse to the input on the box to be extra safe).  This essentially creates power distribution box similar to a breaker box except you are using relays instead of breakers.