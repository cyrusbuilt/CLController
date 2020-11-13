# CLController

ESP8266-based Christmas Lights Controller (Arduino)

The code in this repository represents the firmware for the CLController main module, as well as some components for integrating with [OpenHAB2](https://openhab.org). CLController is a [PlatformIO](https://platformio.org) project and is meant to essentially a multi-module outlet controller, with the idea being that you can control multiple string of traditional Christmas light strings (this is not a pixel controller). Given the hardware design however, it is possible to use this device for multiple applications.  I can be monitored and controlled via [MQTT](https://mqtt.org).  Since there is an [ESP8266](https://www.adafruit.com/product/2471) at the heart of it, that means it is WiFi-enabled and is capable of OTA (Over-the-Air) firmware and configuration updates.

## Theory of Operation

CLController consists of one or more combinations of the following 3 hardware components:
- CLController Main Board
- CLController Relay Module
- CLController I/O Expansion Module

The bare-minimum configuration would be the CLController Main Board and (1) Relay Module.  Each relay module allows control of up to 8 circuits at 10A (MAX) per cicuit. The main control board allows up to (2) Relay Modules to be connected to it providing a total of 16 switched circuits.  You can expand this further by connecting an I/O expansion module.  Each expansion module allows you to daisy chain another expansion module and also supports 2 more relay modules per expansion module. The expansion modules connect using the 2-Wire (I2C) bus an are address configurable (all the main control module is hard-wired for address 0x20).  Each expansion module is auto-detected and intialized during boot.

CLController uses a concept of "Playsheets" which is basically a JSON file containing a collection of sequences that fire the relays in the appropriate order at the appropriate time.  Each "sequence" contains a list of "states" followed by a delay in milliseconds.  Each "state" contains the relay module index, light string index (relay number), and state ("on/off").

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