#ifndef CONFIG_H
#define CONFIG_H

#include <IPAddress.h>

//#define MODEL_1                                     // The Model 1 controller used 5 PowerRelay FeatherWing modules instead of RelayModule over I2C.
#define DEBUG                                       // Enable additional debug output.
#define DEVICE_NAME "CLCONTROLLER"                  // The device name (hostname).
#define BAUD_RATE 115200                            // The BAUD rate (speed) of the serial port (console).
#define ENABLE_OTA                                  // Comment to disable OTA (over-the-air) firmware updates.
#define ENABLE_MDNS                                 // Comment to disable MDNS (multicast-DNS) support.
#define CONFIG_FILE_PATH "/config.json"             // Config file path. Do not alter unless you are sure.
#define PLAYSHEET_FILE_PATH "/playsheet.json"       // Playsheet file path. Do not alter unless you are sure.
#define DEFAULT_SSID "your_ssid_here"               // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "your_wifi_password"       // Put your WiFi password here.
#define CLOCK_TIMEZONE -4                           // The timezone this device is located in. (For example, EST when observing DST = GMT -4, when not = GMT -5).
#define CHECK_WIFI_INTERVAL 30000                   // How often to check WiFi status (millis)
#define CLOCK_SYNC_INTERVAL 3600000                 // How often to sync the local clock with NTP (millis)
#define CHECK_MQTT_INTERVAL 35000                   // MQTT connectivity check interval (millis).
#define MQTT_TOPIC_STATUS "clcontroller/status"     // MQTT status channel to publish to.
#define MQTT_TOPIC_CONTROL "clcontroller/control"   // MQTT control channel to subscribe to.
#define MQTT_BROKER "your_mqtt_broker_ip"           // MQTT broker hostname or IP.
#define MQTT_PORT 8883                              // MQTT port number.
#ifdef ENABLE_OTA
    #include <ArduinoOTA.h>
    #define OTA_HOST_PORT 8266                      // The OTA updater port.
    #define OTA_PASSWORD "your_ota_password_here"   // The OTA updater password.
#endif
const IPAddress defaultIp(192, 168, 0, 215);                     // Default static IP address.
const IPAddress defaultGw(192, 168, 0, 1);                       // Default static gateway address.
const IPAddress defaultSm(255, 255, 255, 0);                     // Default static subnet mask.
const IPAddress defaultDns(192, 168, 0, 1);                      // Default static primary DNS address.

typedef struct {
    // Network stuff
    String hostname;
    String ssid;
    String password;
    IPAddress ip;
    IPAddress gw;
    IPAddress sm;
    IPAddress dns;
    bool useDhcp;

    uint8_t clockTimezone;
    String sheetName;

    // MQTT stuff
    String mqttTopicStatus;
    String mqttTopicControl;
    String mqttBroker;
    String mqttUsername;
    String mqttPassword;
    uint16_t mqttPort;

    // OTA stuff
    uint16_t otaPort;
    String otaPassword;
} config_t;

#endif