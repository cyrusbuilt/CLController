/**
 * @file main.cpp
 * @author Cyrus Brunner (cyrusbuilt at gmail dot com)
 * @brief CLController Firmware
 * 
 * I/O Expansion:
 * 
 * The main controller board has an MCP23017 which provides 16 digital
 * GPIOs and is located at PRIMARY_I2C_ADDRESS. This is what controls the 8 outputs
 * on each of the 2 ports on the main board.  However, it is possible to expand
 * this even further by adding additional MCP23017s (7 additional max) on the I2C bus.
 * As such, the main board provides a 2-pin I2C header that additional expanders to connect to.
 * @version 1.2
 * @date 2022-11-10
 * 
 * @copyright Copyright (c) 2022 Cyrus Brunner
 */

// TODO Possible design changes:
// 1) Change ESP8266 to ESP32
// 2) Split up into further modules - 1] Main controller board. 2] I/O module (MCP3017 and ULN2803). 3] Relay modules. -- Done
// 3) Create abstraction library for relay module (it's essentially an 8bit interface).  -- Done
// 4) Come up with a way to detect relay module presence. -- Done (sort of)
// 12/2/2020 - There is no way to do #4 without adding additional circuitry to the relay module and at least
// one I/O per module for sense.  This can be done by keeping the original design but removing one relay per
// module and using that I/O as an input tied to VSS via a resistor (5.6K ?).  The MCP23017 does not have any
// analog pins, so we can't do an analog read to get resistance. We can only look for low/high. That being said,
// the firmware currently (unreliably) does this by checking to see if the pin on the MCP23017 is LOW after INIT
// (indicating the pin is not floating, the default state).

#include <Arduino.h>
#include <FS.h>
#include <time.h>
#include <vector>
#include "ArduinoJson.h"
#include "ESPCrashMonitor.h"
#include "Console.h"
#include "LED.h"
#include "PubSubClient.h"
#include "ResetManager.h"
#include "TaskScheduler.h"
#include "TelemetryHelper.h"
#include "config.h"
#ifdef MODEL_1
#include "Relay.h"
#else
#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include "RelayModule.h"
#endif
#include "Playsheet.h"

#define FIRMWARE_VERSION "1.2"

// This firmware provides backward compatibility with the original (Model 1)
// CLController board which just consisted of an Adafruit Huzzah ESP8266 and
// 5 Adafruit PowerRelay FeatherWings. By uncommenting the MODEL_1 define in
// config.h, this firmware can be configured for the old controller board.
#ifdef MODEL_1
#define PIN_RELAY_5 12
#define PIN_RELAY_4 13
#define PIN_RELAY_3 14
#define PIN_RELAY_2 16
#define PIN_RELAY_1 15
#define PIN_WIFI_LED 2
#else
#define PRIMARY_I2C_ADDRESS 0
#define I2C_ADDRESS_OFFSET 32
#define PIN_WIFI_LED 16
#define PIN_RUN_LED 14
#define PIN_BUS_ENABLE 12
#endif

#ifdef MODEL_1
enum class RelaySelect : uint8_t {
    RELAY1 = 1,
    RELAY2 = 2,
    RELAY3 = 3,
    RELAY4 = 4,
    RELAY5 = 5
};
#endif

using namespace std;

// Forward declarations.
void operationsLoop();
void onCheckWifi();
void onCheckMqtt();
void onSyncClock();
void onMqttMessage(char* topic, byte* payload, unsigned int length);

// Global vars.
#ifdef ENABLE_MDNS
    #include <ESP8266mDNS.h>
    MDNSResponder mdns;
#endif
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Task tCheckWiFi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWifi);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Task tClockSync(CLOCK_SYNC_INTERVAL, TASK_FOREVER, &onSyncClock);
Scheduler taskMan;
#ifdef MODEL_1
vector<Relay> relays;
#else
bool primaryExpanderFound = false;
vector<byte> devicesFound;
vector<Adafruit_MCP23017> additionalBusses;
vector<RelayModule> relayModules;
Adafruit_MCP23017 primaryBus;
LED runLED(PIN_RUN_LED, NULL);
#endif
LED wifiLED(PIN_WIFI_LED, NULL);
config_t config;
bool filesystemMounted = false;
volatile bool terminateSequence = false;
volatile SystemState sysState = SystemState::BOOTING;

/**
 * @brief Get the current time.
 * 
 * @return String The current date/time in UTC in string format.
 */
String getCurrentTime() {
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    return String(asctime(timeinfo));
}

/**
 * @brief Synchronize the local system clock via NTP. Note: This does not take
 * DST into account. Currently, you will have to adjust the CLOCK_TIMEZONE
 * define manually to account for DST when needed.
 */
void onSyncClock() {
    wifiLED.on();
    configTime(config.clockTimezone * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("INIT: Waiting for NTP time sync...");
    delay(500);
    while (!time(nullptr)) {
        wifiLED.off();
        ESPCrashMonitor.iAmAlive();
        Serial.print(F("."));
        delay(500);
        wifiLED.on();
    }

    wifiLED.off();
    Serial.println(F(" DONE"));
    Serial.print(F("INFO: Current time: "));
    Serial.println(getCurrentTime());
}

/**
 * @brief Publishes the current system state to the configured status topic if
 * connected to the MQTT broker.
 */
void publishSystemState() {
    if (mqttClient.connected()) {
        wifiLED.on();

        DynamicJsonDocument doc(200);
        doc["client_id"] = config.hostname.c_str();
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;
        doc["playingSequence"] = !terminateSequence;
        doc["sheetName"] = config.sheetName;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicStatus.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        wifiLED.off();
    }
}

/**
 * @brief Publishes a device discovery packet to the configured discovery topic.
 */
void publishDiscoveryPacket() {
    if (mqttClient.connected()) {
        wifiLED.on();

        DynamicJsonDocument doc(250);
        doc["name"] = config.hostname;
        doc["class"] = DEVICE_CLASS;
        doc["statusTopic"] = config.mqttTopicStatus;
        doc["controlTopic"] = config.mqttTopicControl;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing discovery packet: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicDiscovery.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        wifiLED.off();
    }
}

/**
 * @brief Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
    wifiLED.off();
    #ifndef MODEL_1
    runLED.off();
    #endif
    sysState = SystemState::NORMAL;
    terminateSequence = false;
    publishSystemState();
}

/**
 * @brief Prints network information details to the serial console.
 */
void printNetworkInfo() {
    Serial.print(F("INFO: Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("INFO: Gateway: "));
    Serial.println(WiFi.gatewayIP());
    Serial.print(F("INFO: Subnet mask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("INFO: DNS server: "));
    Serial.println(WiFi.dnsIP());
    Serial.print(F("INFO: MAC address: "));
    Serial.println(WiFi.macAddress());
    #ifdef DEBUG
        WiFi.printDiag(Serial);
    #endif
}

/**
 * @brief Scan for available networks and dump each discovered network to the
 * console.
 */
void getAvailableNetworks() {
    ESPCrashMonitor.defer();
    Serial.println(F("INFO: Scanning WiFi networks..."));
    int numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        Serial.print(F("ID: "));
        Serial.print(i);
        Serial.print(F("\tNetwork name: "));
        Serial.print(WiFi.SSID(i));
        Serial.print(F("\tSignal strength:"));
        Serial.println(WiFi.RSSI(i));
    }
    Serial.println(F("----------------------------------"));
}

/**
 * @brief Resets the MCP23017 I2C GPIO expander.
 */
void busReset() {
    Serial.println(F("WARN: Performing bus reset ..."));
    digitalWrite(PIN_BUS_ENABLE, LOW);
    delay(500);
    digitalWrite(PIN_BUS_ENABLE, HIGH);
    delay(500);
}

/**
 * @brief Performs a soft-reboot.
 */
void reboot() {
    busReset();
    Serial.println(F("INFO: Rebooting..."));
    Serial.flush();
    delay(1000);
    ResetManager.softReset();
}

/**
 * @brief Persists the current running configuration in memory to the config.json
 * file in flash.
 */
void saveConfiguration() {
    Serial.print(F("INFO: Saving configuration to "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.println(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mount."));
        return;
    }

    StaticJsonDocument<350> doc;
    doc["hostname"] = config.hostname;
    doc["useDhcp"] = config.useDhcp;
    doc["ip"] = config.ip.toString();
    doc["gateway"] = config.gw.toString();
    doc["subnetmask"] = config.sm.toString();
    doc["dnsServer"] = config.dns.toString();
    doc["wifiSSID"] = config.ssid;
    doc["wifiPassword"] = config.password;
    doc["mqttBroker"] = config.mqttBroker;
    doc["mqttPort"] = config.mqttPort;
    doc["mqttControlTopic"] = config.mqttTopicControl;
    doc["mqttStatusTopic"] = config.mqttTopicStatus;
    doc["mqttDiscoveryTopic"] = config.mqttTopicDiscovery;
    doc["mqttUsername"] = config.mqttUsername;
    doc["mqttPassword"] = config.mqttPassword;
    #ifdef ENABLE_OTA
        doc["otaPort"] = config.otaPort;
        doc["otaPassword"] = config.otaPassword;
    #endif

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "w");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to open config file for writing."));
        doc.clear();
        return;
    }

    serializeJsonPretty(doc, configFile);
    doc.clear();
    configFile.flush();
    configFile.close();
    Serial.println(F("DONE"));
}

/**
 * @brief Prints a warning to the console.
 * 
 * @param message The message to print.
 */
void printWarningAndContinue(const __FlashStringHelper *message) {
    Serial.println();
    Serial.println(message);
    Serial.print(F("INFO: Continuing... "));
}

/**
 * @brief Sets the running configuration in memory to the factory defaults.
 */
void setConfigurationDefaults() {
    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = defHostname;
    config.ip = defaultIp;
    config.mqttBroker = MQTT_BROKER;
    config.mqttPassword = "";
    config.mqttPort = MQTT_PORT;
    config.mqttTopicControl = MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = MQTT_TOPIC_STATUS;
    config.mqttTopicDiscovery = MQTT_TOPIC_DISCOVERY;
    config.mqttUsername = "";
    config.password = DEFAULT_PASSWORD;
    config.sm = defaultSm;
    config.ssid = DEFAULT_SSID;
    config.useDhcp = false;
    config.clockTimezone = CLOCK_TIMEZONE;
    config.dns = defaultDns;
    config.gw = defaultGw;
    config.sheetName = "";

    #ifdef ENABLE_OTA
        config.otaPassword = OTA_PASSWORD;
        config.otaPort = OTA_HOST_PORT;
    #endif
}

/**
 * @brief Loads the configuration from the config.json file in local flash
 * to the running configuration in memory.
 */
void loadConfiguration() {
    memset(&config, 0, sizeof(config));

    Serial.print(F("INFO: Loading config file "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.print(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    if (!SPIFFS.exists(CONFIG_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("WARN: Config file does not exist. Creating with default config ..."));
        saveConfiguration();
        return;
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "r");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to open config file. Using default config."));
        return;
    }

    size_t size = configFile.size();
    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
    if (size > freeMem) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: Not enough free memory to load document. Size = "));
        Serial.print(size);
        Serial.print(F(", Free = "));
        Serial.println(freeMem);
        configFile.close();
        return;
    }

    DynamicJsonDocument doc(freeMem);
    DeserializationError error = deserializeJson(doc, configFile);
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Fail to parse config file to JSON. Using default config."));
        configFile.close();
        return;
    }

    doc.shrinkToFit();
    configFile.close();

    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = doc.containsKey("hostname") ? doc["hostname"].as<String>() : defHostname;
    config.useDhcp = doc.containsKey("isDhcp") ? doc["isDhcp"].as<bool>() : false;
    
    if (doc.containsKey("ip")) {
        if (!config.ip.fromString(doc["ip"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.ip = defaultIp;
    }

    if (doc.containsKey("gateway")) {
        if (!config.gw.fromString(doc["gateway"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
        }
    }
    else {
        config.gw = defaultGw;
    }

    if (doc.containsKey("subnetmask")) {
        if (!config.sm.fromString(doc["subnetmask"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid subnet mask in configuration. Falling back to factory default."));
        }
    }
    else {
        config.sm = defaultSm;
    }

    if (doc.containsKey("dns")) {
        if (!config.dns.fromString(doc["dns"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid DNS IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.dns = defaultDns;
    }

    config.ssid = doc.containsKey("wifiSSID") ? doc["wifiSSID"].as<String>() : DEFAULT_SSID;
    config.password = doc.containsKey("wifiPassword") ? doc["wifiPassword"].as<String>() : DEFAULT_PASSWORD;
    config.mqttBroker = doc.containsKey("mqttBroker") ? doc["mqttBroker"].as<String>() : MQTT_BROKER;
    config.mqttPort = doc.containsKey("mqttPort") ? doc["mqttPort"].as<int>() : MQTT_PORT;
    config.mqttTopicControl = doc.containsKey("mqttControlTopic") ? doc["mqttControlTopic"].as<String>() : MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = doc.containsKey("mqttStatusTopic") ? doc["mqttStatusTopic"].as<String>() : MQTT_TOPIC_STATUS;
    config.mqttTopicDiscovery = doc.containsKey("mqttDiscoveryTopic") ? doc["mqttDiscoveryTopic"].as<String>() : MQTT_TOPIC_DISCOVERY;
    config.mqttUsername = doc.containsKey("mqttUsername") ? doc["mqttUsername"].as<String>() : "";
    config.mqttPassword = doc.containsKey("mqttPassword") ? doc["mqttPassword"].as<String>() : "";

    #ifdef ENABLE_OTA
        config.otaPort = doc.containsKey("otaPort") ? doc["otaPort"].as<uint16_t>() : MQTT_PORT;
        config.otaPassword = doc.containsKey("otaPassword") ? doc["otaPassword"].as<String>() : OTA_PASSWORD;
    #endif

    doc.clear();
    Serial.println(F("DONE"));
}

/**
 * @brief Performs a "factory restore" of the running configuration. This is
 * done by first deleting the config.json file in flash and rebooting the
 * MCU. On startup, the running config will fallback to the factory defaults
 * which will the then be persisted to a new config.json file.
 */
void doFactoryRestore() {
    Serial.println();
    Serial.println(F("Are you sure you wish to restore to factory default? (Y/n)"));
    Console.waitForUserInput();
    
    String str = Console.getInputString();
    if (str == "Y" || str == "y") {
        Serial.print(F("INFO: Clearing current config... "));
        if (filesystemMounted) {
            if (SPIFFS.remove(CONFIG_FILE_PATH)) {
                Serial.println(F("DONE"));
                Serial.print(F("INFO: Removed file: "));
                Serial.println(CONFIG_FILE_PATH);

                Serial.print(F("INFO: Rebooting in "));
                for (uint8_t i = 5; i >= 1; i--) {
                    Serial.print(i);
                    Serial.print(F(" "));
                    delay(1000);
                }

                reboot();
            }
            else {
                Serial.println(F("FAIL"));
                Serial.println(F("ERROR: Failed to delete cofiguration file."));
            }
        }
        else {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Filesystem not mounted."));
        }
    }

    Serial.println();
}

/**
 * @brief Loads the "play sheet" into memory.
 */
void loadSequenceSheet() {
    Serial.print(F("INFO: Loading sequence file: "));
    Serial.print(PLAYSHEET_FILE_PATH);
    Serial.print(F(" ..."));

    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        terminateSequence = true;
        return;
    }

    if (!SPIFFS.exists(PLAYSHEET_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Sequence file does not exist."));
        terminateSequence = true;
        return;
    }

    File sequenceFile = SPIFFS.open(PLAYSHEET_FILE_PATH, "r");
    if (!sequenceFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to open sequence file."));
        terminateSequence = true;
        return;
    }

    // Do we have enough free contiguous memory to load the file?
    size_t size = sequenceFile.size();
    #ifdef DEBUG
        Serial.println();
        Serial.print(F("DEBUG: Playsheet file size: "));
        Serial.println(size);
    #endif
    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
    #ifdef DEBUG
        Serial.print(F("DEBUG: Free memory before load: "));
        Serial.println(freeMem);
    #endif
    if (size > freeMem) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: Not enough free memory to load document. Size = "));
        Serial.print(size);
        Serial.print(F(", Free = "));
        Serial.println(freeMem);
        sequenceFile.close();
        return;
    }

    DynamicJsonDocument doc(freeMem);
    DeserializationError error = deserializeJson(doc, sequenceFile);
    if (error) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: Failed to parse sequence file to JSON: "));
        Serial.println(error.c_str());
        sequenceFile.close();
        return;
    }

    sequenceFile.close();
    doc.shrinkToFit();
    
    config.sheetName = doc["name"].as<String>();
    Playsheet.sheetName = config.sheetName.c_str();
    
    #ifdef MODEL_1
    RelayState newState = RelayOpen;
    const uint8_t modIdx = 0;
    #else
    uint8_t modIdx = 0;
    ModuleRelayState newState = ModuleRelayState::OPEN;
    #endif
    RelaySelect relay = RelaySelect::RELAY1;

    JsonArray sequences = doc["seq"].as<JsonArray>();
    for (auto sequence : sequences) {
        Sequence seq;
        JsonArray states = sequence["states"];
        for (auto state : states) {
            // We don't care about the module index if we're running on a Model 1 controller.
            #ifndef MODEL_1
            modIdx = state["modIdx"].as<uint8_t>();
            #endif
            if (strcmp(state["lsState"].as<const char*>(), "on") == 0) {
                #ifdef MODEL_1
                newState = RelayClosed;
                #else
                newState = ModuleRelayState::CLOSED;
                #endif
            }
            else {
                #ifdef MODEL_1
                newState = RelayOpen;
                #else
                newState = ModuleRelayState::OPEN;
                #endif
            }

            relay = (RelaySelect)state["lsIdx"].as<uint8_t>();
            #ifdef MODEL_1
            SequenceState theState(modIdx, newState, (uint8_t)relay);
            #else
            SequenceState theState(modIdx, newState, relay);
            #endif
            seq.states.push_back(theState);
        }

        seq.delay = sequence["delayMs"].as<unsigned long>();
        Playsheet.sequences.push_back(seq);
    }

    doc.clear();
    #ifdef DEBUG
        freeMem = ESP.getMaxFreeBlockSize() - 512;
        Serial.print(F("DEBUG: Free memory after load: "));
        Serial.println(freeMem);
    #endif
    Serial.println(F("DONE"));
}

/**
 * @brief Executes the "play sheet". This will run continuously until stopped.
 */
void playSequenceSheet() {
    if (terminateSequence) {
        return;
    }

    Serial.print(F("INFO: Executing sequence sheet: "));
    Serial.println(Playsheet.sheetName);

    size_t count = Playsheet.sequences.size();
    Serial.print(F("INFO: Sequence count: "));
    Serial.println(count);
    for (auto seq = Playsheet.sequences.begin(); seq != Playsheet.sequences.end(); seq++) {
        #ifndef MODEL_1
        if (runLED.isOff()) {
            runLED.on();
        }
        #endif

        // This will introduce slight delays during sequences, but it beats blocking
        // other operations from processing and keeps the watchdog fed.
        // The ESP-32 would probably be better suited for this since we can run a
        // a second thread for running sequences and keep everything else
        // on the main loop.  The alternative would be to switch to using
        // the ESP8266 RTOS SDK. But that would mean refactoring the entire
        // project as we would no longer be using the Arduino Core (the Arduino
        // core for ESP8266 currently uses the non-RTOS SDK under the hood).
        // Each dependency would need to be evaluated to see which (if any)
        // are compatible and find replacements or write new ones from scratch.
        // I'd rather not do all that yet.
        Serial.println(F("DEBUG: OpLoop"));
        operationsLoop();
        if (terminateSequence) {
            break;
        }

        #ifdef DEBUG
        Serial.print(F("DEBUG: seq delay = "));
        Serial.print(seq->delay);
        Serial.print(F(", state count = "));
        Serial.println(seq->states.size());
        #endif
        for (auto state = seq->states.begin(); state != seq->states.end(); state++) {
            #ifdef DEBUG
            // TODO Assuming we can get detection working correctly,
            // TODO we should probably skip any modules that don't actually exist.
            Serial.print(F("DEBUG: State modIdx = "));
            Serial.print(state->moduleIndex);
            Serial.print(F(", lsIdx = "));
            Serial.print((uint8_t)state->lightStringIndex);
            Serial.print(F(", lsState = "));
            Serial.println((uint8_t)state->lightStringState);
            #endif
            #ifdef MODEL_1
            relays.at(state->lightStringIndex - 1).setState(state->lightStringState);
            #else
            relayModules.at(state->moduleIndex).setState(state->lightStringIndex, state->lightStringState);
            #endif
        }

        // delay for note duration + 30%
        delay(seq->delay * 1.30);
    }

    #ifndef MODEL_1
    runLED.off();
    #endif
    Serial.print(F("INFO: Finished running sequence: "));
    Serial.println(Playsheet.sheetName);
}

/**
 * @brief Reconnects to the MQTT broker if needed.
 * 
 * @return true If successfully reconnected or already connected.
 * @return false If not connected and could not reconnect.
 */
bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
        wifiLED.on();
        Serial.print(F("INFO: Attempting to establish MQTT connection to "));
        Serial.print(config.mqttBroker);
        Serial.print(F(" on port "));
        Serial.print(config.mqttPort);
        Serial.println(F(" ... "));

        bool didConnect = false;
        if (config.mqttUsername.length() > 0 && config.mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(config.hostname.c_str(), config.mqttUsername.c_str(), config.mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(config.hostname.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to topic: "));
            Serial.println(config.mqttTopicControl);
            mqttClient.subscribe(config.mqttTopicControl.c_str());

            Serial.print(F("INFO: Publishing to topic: "));
            Serial.println(config.mqttTopicStatus);

            Serial.print(F("INFO: Discovery topic: "));
            Serial.println(config.mqttTopicDiscovery);
        }
        else {
            String failReason = TelemetryHelper::getMqttStateDesc(mqttClient.state());
            Serial.print(F("ERROR: Failed to connect to MQTT broker: "));
            Serial.println(failReason);
            return false;
        }

        wifiLED.off();
    }

    return true;
}

/**
 * @brief Callback task method to check the MQTT connection and reconnect if
 * needed, then publish the system status and discovery packet.
 */
void onCheckMqtt() {
    Serial.println(F("INFO: Checking MQTT connections status..."));
    if (reconnectMqttClient()) {
        Serial.println(F("INFO: Successfully reconnected to MQTT broker."));
        publishSystemState();
        publishDiscoveryPacket();
    }
    else {
        Serial.println(F("ERROR: MQTT connection lost and reconnect failed."));
        Serial.print(F("INFO: Retrying connection in "));
        Serial.print(CHECK_MQTT_INTERVAL % 1000);
        Serial.println(F(" seconds ..."));
    }
}

/**
 * @brief Turns all relays on all relay modules on.
 */
void allRelaysOn() {
    uint8_t index = 0;
    #ifdef MODEL_1
    for (auto relayModule = relays.begin(); relayModule != relays.end(); relayModule++) {
        Serial.print(F("INFO: Turning on relay "));
        Serial.println(index + 1);
        relayModule->close();
        index++;
    }
    #else
    for (auto module = relayModules.begin(); module != relayModules.end(); module++) {
        Serial.print(F("INFO: Turning all relays on module at address "));
        Serial.print(index);
        Serial.println(F(" on ..."));
        module->allRelaysClosed();
        index++;
    }
    #endif
}

/**
 * @brief Turns off all relays on all relay modules.
 */
void allRelaysOff() {
    uint8_t index = 0;
    #ifdef MODEL_1
    for (auto relayModule = relays.begin(); relayModule != relays.end(); relayModule++) {
        Serial.print(F("INFO: Turning of relay "));
        Serial.println(index + 1);
        relayModule->open();
        index++;
    }
    #else
    for (auto module = relayModules.begin(); module != relayModules.end(); module++) {
        Serial.print(F("INFO: Turning all relays on module at address "));
        Serial.print(index);
        Serial.println(F(" off ..."));
        module->allRelaysOpen();
        index++;
    }
    #endif
}

/**
 * @brief Handles control requests. This executes the given command not the
 * system is not currently disabled. If disabled, then only allowable command
 * is "ENABLE". Unrecognized commands are ignored.
 * 
 * @param cmd The command to handle.
 */
void handleControlRequest(ControlCommand cmd) {
    if (sysState == SystemState::DISABLED && cmd != ControlCommand::ENABLE) {
        // THOU SHALT NOT PASS!!! 
        // We can't process this command because we are disabled.
        Serial.print(F("WARN: Ingoring command "));
        Serial.print((uint8_t)cmd);
        Serial.print(F(" because the system is currently disabled."));
        return;
    }

    switch (cmd) {
        case ControlCommand::ENABLE:
            Serial.println(F("INFO: Enabling system."));
            sysState = SystemState::NORMAL;
            break;
        case ControlCommand::DISABLE:
            Serial.println(F("WARN: Disabling system."));
            sysState = SystemState::DISABLED;
            break;
        case ControlCommand::PAUSE_SEQUENCE:
            Serial.println(F("WARN: Interrupting light sequence."));
            terminateSequence = true;
            break;
        case ControlCommand::PLAY_SEQUENCE:
            terminateSequence = false;
            break;
        case ControlCommand::REBOOT:
            reboot();
            break;
        case ControlCommand::REQUEST_STATUS:
            break;
        case ControlCommand::ALL_ON:
            Serial.println(F("WARN: Interrupting light sequence."));
            terminateSequence = true;
            allRelaysOn();
            break;
        case ControlCommand::ALL_OFF:
            Serial.println(F("WARN: Interrupting light sequence."));
            terminateSequence = true;
            allRelaysOff();
        default:
            Serial.print(F("WARN: Unknown command: "));
            Serial.println((uint8_t)cmd);
            break;
    }

    publishSystemState();
}

/**
 * @brief Callback handler method that gets called whenever a message is
 * received from the configured control topic this firmware is subscribed to.
 * This attempts to parse the message and the message was meant for this
 * device, will attempt to execute the control request that was parsed.
 * 
 * @param topic The topic the message was received on.
 * @param payload The message payload.
 * @param length The length of the payload.
 */
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print(F("INFO: [MQTT] Message arrived: ["));
    Serial.print(topic);
    Serial.print(F("] "));

    // It's a lot easier to deal with if we just convert the payload
    // to a string first.
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.println(msg);

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg.c_str());
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

    if (doc.containsKey("client_id")) {
        String id = doc["client_id"].as<String>();
        id.toUpperCase();
        if (!id.equals(config.hostname)) {
            Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
            doc.clear();
            return;
        }
    }
    else {
        Serial.println(F("WARN: MQTT message does not contain client ID. Ignoring..."));
        doc.clear();
        return;
    }

    if (!doc.containsKey("command")) {
        Serial.println(F("WARN: MQTT message does not contain a control command. Ignoring..."));
        doc.clear();
        return;
    }

    // When system is the "disabled" state, the only command it will accept
    // is "enable". All other commands are ignored.
    ControlCommand cmd = (ControlCommand)doc["command"].as<uint8_t>();

    doc.clear();
    handleControlRequest(cmd);
}

/**
 * @brief Enter fail-safe mode. This will suspend all tasks, disable relay
 * activation, and propmpt the user for configuration.
 */
void failSafe() {
    sysState = SystemState::DISABLED;
    terminateSequence = true;
    publishSystemState();
    ESPCrashMonitor.defer();
    Serial.println();
    Serial.println(F("ERROR: Entering failsafe (config) mode..."));
    taskMan.disableAll();
    #ifndef MODEL_1
    runLED.off();
    #endif
    wifiLED.on();
    Console.enterCommandInterpreter();
}

/**
 * @brief Initializes the MDNS responder (if enabled).
 */
void initMDNS() {
    #ifdef ENABLE_MDNS
        Serial.print(F("INIT: Starting MDNS responder... "));
        if (WiFi.status() == WL_CONNECTED) {
            ESPCrashMonitor.defer();
            delay(500);

            if (!mdns.begin(config.hostname)) {
                Serial.println(F(" FAILED"));
                return;
            }

            #ifdef ENABLE_OTA
                mdns.addService(config.hostname, "ota", config.otaPort);
            #endif
            Serial.println(F(" DONE"));
        }
        else {
            Serial.println(F(" FAILED"));
        }
    #endif
}

/**
 * @brief Initialize the SPIFFS filesystem.
 */
void initFilesystem() {
    Serial.print(F("INIT: Initializing SPIFFS and mounting filesystem... "));
    if (!SPIFFS.begin()) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to mount filesystem."));
        return;
    }

    filesystemMounted = true;
    Serial.println(F("DONE"));
    setConfigurationDefaults();
    loadConfiguration();
    loadSequenceSheet();
}

/**
 * @brief Initializes the MQTT client.
 */
void initMQTT() {
    Serial.print(F("INIT: Initializing MQTT client... "));
    mqttClient.setServer(config.mqttBroker.c_str(), config.mqttPort);
    mqttClient.setCallback(onMqttMessage);
    Serial.println(F("DONE"));
    if (reconnectMqttClient()) {
        delay(500);
        publishSystemState();
    }
}

/**
 * @brief Attempt to connect to the configured WiFi network. This will break
 * any existing connection first.
 */
void connectWifi() {
    if (config.hostname) {
        WiFi.hostname(config.hostname.c_str());
    }

    Serial.println(F("DEBUG: Setting mode..."));
    WiFi.mode(WIFI_STA);
    Serial.println(F("DEBUG: Disconnect and clear to prevent auto connect..."));
    WiFi.persistent(false);
    WiFi.disconnect(true);
    ESPCrashMonitor.defer();

    delay(1000);
    if (config.useDhcp) {
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        WiFi.config(config.ip, config.gw, config.sm, config.gw);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(config.ssid, config.password);
    Serial.println(F("DEBUG: Waiting for connection..."));
    
    const int maxTries = 20;
    int currentTry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (currentTry < maxTries)) {
        ESPCrashMonitor.iAmAlive();
        currentTry++;
        wifiLED.blink(500);
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        // Connection failed. Maybe the AP went down? Let's try again later.
        Serial.println(F("ERROR: Failed to connect to WiFi!"));
        Serial.println(F("WARN: Will attempt to reconnect at scheduled interval."));
    }
    else {
        printNetworkInfo();
    }
}

/**
 * @brief Scans the I2C bus for expansion modules.
 */
#ifndef MODEL_1
void scanBusDevices() {
    byte error;
    byte address;
    int devices = 0;

    // NOTE: We can have a max of only (8) MCP23017 chips connected to the bus at a time.
    // This gives a range of addresses 0 - 7 (which really translates to 32 - 39).
    Serial.println(F("INFO: Beginning I2C bus scan ..."));
    for (address = 32; address < 40; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            devices++;
            Serial.print(F("INFO: I2C device found at address 0x"));
            if (address < 16) {
                Serial.print(F("0"));
            }

            Serial.print(address, HEX);
            Serial.println(F("!"));
            uint8_t realAddress = address - I2C_ADDRESS_OFFSET;
            if (!primaryExpanderFound && realAddress == PRIMARY_I2C_ADDRESS) {
                primaryExpanderFound = true;
            }
            else {
                devicesFound.push_back(realAddress);
            }
        }
        else if (error == 4) {
            Serial.print(F("ERROR: Unknown error at address 0x"));
            if (address < 16) {
                Serial.print(F("0"));
            }

            Serial.println(address, HEX);
        }
    }

    if (devices == 0) {
        Serial.println(F("ERROR: No devices found!"));
    }
    else {
        Serial.println(F("INFO: Scan complete."));
    }
}
#endif

/**
 * @brief Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to SSID: "));
    Serial.print(config.ssid);
    Serial.println(F("..."));
    
    connectWifi();
}

/**
 * @brief Initializes the OTA update listener if enabled.
 */
void initOTA() {
    #ifdef ENABLE_OTA
        Serial.print(F("INIT: Starting OTA updater... "));
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setPort(config.otaPort);
            ArduinoOTA.setHostname(config.hostname.c_str());
            ArduinoOTA.setPassword(config.otaPassword.c_str());
            ArduinoOTA.onStart([]() {
                // Handles start of OTA update. Determines update type.
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                }
                else {
                    type = "filesystem";
                }

                sysState = SystemState::UPDATING;
                publishSystemState();
                Serial.println("INFO: Starting OTA update (type: " + type + ") ...");
            });
            ArduinoOTA.onEnd([]() {
                // Handles update completion.
                Serial.println(F("INFO: OTA updater stopped."));
            });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
                // Reports update progress.
                wifiLED.blink(100);
                ESPCrashMonitor.iAmAlive();
                Serial.printf("INFO: OTA Update Progress: %u%%\r", (progress / (total / 100)));
            });
            ArduinoOTA.onError([](ota_error_t error) {
                // Handles OTA update errors.
                Serial.printf("ERROR: OTA update error [%u]: ", error);
                switch(error) {
                    case OTA_AUTH_ERROR:
                        Serial.println(F("Auth failed."));
                        break;
                    case OTA_BEGIN_ERROR:
                        Serial.println(F("Begin failed."));
                        break;
                    case OTA_CONNECT_ERROR:
                        Serial.println(F("Connect failed."));
                        break;
                    case OTA_RECEIVE_ERROR:
                        Serial.println(F("Receive failed."));
                        break;
                    case OTA_END_ERROR:
                        Serial.println(F("End failed."));
                        break;
                }
            });
            ArduinoOTA.begin();
            Serial.println(F("DONE"));
        }
        else {
            Serial.println(F("FAIL"));
        }
    #endif
}

/**
 * @brief Callback routine for checking WiFi connectivity.
 */
void onCheckWifi() {
    Serial.println(F("INFO: Checking WiFi connectivity..."));
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WARN: Lost connection. Attempting reconnect..."));
        connectWifi();
        if (WiFi.status() == WL_CONNECTED) {
            initMDNS();
            initOTA();
        }
    }
}

/**
 * @brief Initializes the local RS-232 serial interface.
 */
void initSerial() {
    Serial.begin(BAUD_RATE);
    #ifdef DEBUG
        const bool debug = true;
    #else
        const bool debug = false;
    #endif
    Serial.setDebugOutput(debug);
    Serial.println();
    Serial.print(F("CLController v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" booting ..."));
}

/**
 * @brief Initializes the I2C communication bus, scans for devices, then
 * attempts to initialize any detected expansion modules.
 */
#ifndef MODEL_1
void initComBus() {
    Serial.println(F("INIT: Initializing communication bus ..."));

    // Turn on the MCP23017.
    pinMode(PIN_BUS_ENABLE, OUTPUT);
    digitalWrite(PIN_BUS_ENABLE, HIGH);
    delay(500);

    // Init I2C bus and start scanning for devices.
    Wire.begin();
    scanBusDevices();
    if (primaryExpanderFound) {
        Serial.println(F("INFO: Found primary host bus controller."));
        Serial.print(F("INFO: Secondary bus controllers found: "));
        Serial.println(devicesFound.size());
        
        primaryBus.begin((uint8_t)PRIMARY_I2C_ADDRESS);
        additionalBusses.push_back(primaryBus);  // The primary bus controller should always be at index 0.

        for (std::size_t i = 0; i < devicesFound.size(); i++) {
            Adafruit_MCP23017 newBus;
            newBus.begin(devicesFound.at(i));
            additionalBusses.push_back(newBus);
        }
    }
    else {
        terminateSequence = true;
        Serial.println(F("ERROR: Primary host bus controller not found!!"));
        // TODO HCF???
    }

    Serial.println(F("INIT: Comm bus initialization complete."));
}
#endif

/**
 * @brief Initializes all known relay modules.
 */
void initRelayModules() {
    Serial.println(F("INIT: Initializing relay modules ..."));

    #ifdef MODEL_1
    const uint8_t pins[5] = {
        PIN_RELAY_1,
        PIN_RELAY_2,
        PIN_RELAY_3,
        PIN_RELAY_4,
        PIN_RELAY_5
    };

    for (uint8_t i = 0; i < 5; i++) {
        String name = "RELAY_" + String(i);
        Relay newRelay(pins[i], NULL, name.c_str());
        newRelay.init();
        relays.push_back(newRelay);
        Serial.print(F("INIT: Configured relay "));
        Serial.println(name);
    }
    #else
    if (!primaryExpanderFound) {
        // NOTE: I actually struggle with this one. My gut says if we can't even detect
        // the onboard I/O expander, then we probably should just fail. On the other hand,
        // what if the onboard chip failed or was removed, but the user is still trying
        // to use the main board with an expansion module and doesn't care that that the
        // onboard expander is missing (or toast)? Should we still allow the system to
        // continue working with *any* I/O expander we can detect? Or continue to assume
        // that if the primary failed, we probably have bigger problems but force it
        // to be a condition to function that the primary chip be present even if the
        // playsheet doesn't use it?
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: No bus controller found."));
        return;
    }

    Serial.print(F("INIT: Init relay modules A and B at bus address: "));
    Serial.println(PRIMARY_I2C_ADDRESS, HEX);
    RelayModule moduleA(&primaryBus, RelayModulePort::PORTA);
    if (moduleA.detect()) {
        Serial.println(F("INFO: Detected relay module 0 (port A, bus 0)"));
        moduleA.init();
        relayModules.push_back(moduleA);
    }

    RelayModule moduleB(&primaryBus, RelayModulePort::PORTB);
    if (moduleB.detect()) {
        Serial.println(F("INFO: Detected relay module 1 (port B, bus 0)"));
        moduleB.init();
        relayModules.push_back(moduleB);
    }

    // Skip ahead by 1 since the first bus is always the primary.
    for (size_t i = 1; i < additionalBusses.size(); i++) {
        Serial.print(F("INIT: Init relay modules A and B at bus address: "));
        Serial.println(devicesFound.at(i), HEX);
        Adafruit_MCP23017 bus = additionalBusses.at(i);

        RelayModule newModuleA(&bus, RelayModulePort::PORTA);
        if (newModuleA.detect()) {
            Serial.print(F("INFO: Detected relay module "));
            Serial.print(i + 1);
            Serial.print(F(" (port A, bus "));
            Serial.print(i);
            Serial.println(F(")"));
            newModuleA.init();
            relayModules.push_back(newModuleA);
        }

        RelayModule newModuleB(&bus, RelayModulePort::PORTB);
        if (newModuleB.detect()) {
            Serial.print(F("INFO: Detected relay module "));
            Serial.print(i + 2);
            Serial.print(F(" (port B, bus "));
            Serial.print(i);
            Serial.println(F(")"));
            newModuleB.init();
            relayModules.push_back(newModuleB);
        }
    }
    #endif

    Serial.println(F("INIT: Relay module initialization complete."));
}

/**
 * @brief Initializes outputs (status LEDs).
 */
void initOutputs() {
    Serial.print(F("INIT: Initializing outputs ..."));
    wifiLED.init();
    wifiLED.on();
    #ifndef MODEL_1
    runLED.init();
    runLED.on();
    #endif
    Serial.println(F("DONE"));
}

/**
 * @brief Callback handler method for running the "Play sheet" from the CLI.
 */
void handleRunPlaysheet() {
    terminateSequence = false;
}

/**
 * @brief Callback handler method for interrupting the "Play sheet" execution
 * from the CLI.
 */
void handleInterruptPlaysheet() {
    Serial.println(F("INFO: Playsheet interrupt..."));
    terminateSequence = true;
}

/**
 * @brief Callback handler method for setting the new hostname from the CLI.
 * 
 * @param newHostname The new hostname to set.
 */
void handleNewHostname(const char* newHostname) {
    if (config.hostname != newHostname) {
        config.hostname = newHostname;
        initMDNS();
    }
}

/**
 * @brief Callback handler method for switching to DHCP mode from the CLI.
 */
void handleSwitchToDhcp() {
    if (config.useDhcp) {
        Serial.println(F("INFO: DHCP mode already set. Skipping..."));
        Serial.println();
    }
    else {
        config.useDhcp = true;
        Serial.println(F("INFO: Set DHCP mode."));
        WiFi.config(0U, 0U, 0U, 0U);
    }
}

/**
 * @brief Callback handler method for switching to static IP mode from the CLI.
 * 
 * @param newIp The new IP to use.
 * @param newSm The subnet mask to use.
 * @param newGw The gateway IP to use.
 * @param newDns The DNS IP to use.
 */
void handleSwitchToStatic(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns) {
    config.ip = newIp;
    config.sm = newSm;
    config.gw = newGw;
    config.dns = newDns;
    Serial.println(F("INFO: Set static network config."));
    WiFi.config(config.ip, config.gw, config.sm, config.dns);
}

/**
 * @brief Callback handler method for reconnect to the WiFi network from the CLI.
 */
void handleReconnectFromConsole() {
    // Attempt to reconnect to WiFi.
    onCheckWifi();
    if (WiFi.status() == WL_CONNECTED) {
        printNetworkInfo();
        resumeNormal();
    }
    else {
        Serial.println(F("ERROR: Still no network connection."));
        Console.enterCommandInterpreter();
    }
}

/**
 * @brief Callback handler method for configuring the WiFi connection from the
 * CLI.
 * @param newSsid The new SSID to use. 
 * @param newPassword The new password to use.
 */
void handleWifiConfig(String newSsid, String newPassword) {
    if (config.ssid != newSsid || config.password != newPassword) {
        config.ssid = newSsid;
        config.password = newPassword;
        connectWifi();
    }
}

/**
 * @brief Callback handler method for saving the current running configuration
 * to the config.json file in flash. This will also trigger a WiFi reconnect.
 */
void handleSaveConfig() {
    saveConfiguration();
    WiFi.disconnect(true);
    onCheckWifi();
}

/**
 * @brief Callback handler method for configuring MQTT from the CLI. This will
 * cause an MQTT reconnect.
 * 
 * @param newBroker The new MQTT broker.
 * @param newPort The new port.
 * @param newUsername The new username.
 * @param newPassw The new password.
 * @param newConChan The new control topic.
 * @param newStatChan The new status topic.
 */
void handleMqttConfigCommand(String newBroker, int newPort, String newUsername, String newPassw, String newConChan, String newStatChan) {
    mqttClient.unsubscribe(config.mqttTopicControl.c_str());
    mqttClient.disconnect();

    config.mqttBroker = newBroker;
    config.mqttPort = newPort;
    config.mqttUsername = newUsername;
    config.mqttPassword = newPassw;
    config.mqttTopicControl = newConChan;
    config.mqttTopicStatus = newStatChan;

    initMQTT();
    Serial.println();
}

/**
 * @brief 
 * 
 */
void handleRunTestSequence() {
    // TODO How to do this?
}

/**
 * @brief Initializes the CLI.
 */
void initConsole() {
    Serial.print(F("INIT: Initializing console... "));

    Console.setHostname(config.hostname);
    Console.setMqttConfig(
        config.mqttBroker,
        config.mqttPort,
        config.mqttUsername,
        config.mqttPassword,
        config.mqttTopicControl,
        config.mqttTopicStatus
    );
    Console.onRunPlaysheet(handleRunPlaysheet);
    Console.onInterruptPlaysheet(handleInterruptPlaysheet);
    Console.onRebootCommand(reboot);
    Console.onScanNetworks(getAvailableNetworks);
    Console.onFactoryRestore(doFactoryRestore);
    Console.onHostnameChange(handleNewHostname);
    Console.onDhcpConfig(handleSwitchToDhcp);
    Console.onStaticConfig(handleSwitchToStatic);
    Console.onReconnectCommand(handleReconnectFromConsole);
    Console.onWifiConfigCommand(handleWifiConfig);
    Console.onSaveConfigCommand(handleSaveConfig);
    Console.onMqttConfigCommand(handleMqttConfigCommand);
    Console.onConsoleInterrupt(failSafe);
    Console.onAllLightsOn(allRelaysOn);
    Console.onAllLightsOff(allRelaysOff);
    Console.onResumeCommand(resumeNormal);

    Serial.println(F("DONE"));
}

/**
 * @brief Initializes the task manager and all recurring tasks.
 */
void initTaskManager() {
    Serial.print(F("INIT: Initializing task scheduler... "));

    taskMan.init();
    taskMan.addTask(tCheckWiFi);
    taskMan.addTask(tCheckMqtt);
    taskMan.addTask(tClockSync);
    
    tCheckWiFi.enableDelayed(30000);
    tCheckMqtt.enableDelayed(1000);
    tClockSync.enable();
    Serial.println(F("DONE"));
}

/**
 * @brief Initializes the crash monitor and dump any previous crash data to the
 * serial console.
 */
void initCrashMonitor() {
    Serial.print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    Serial.println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
}

/**
 * @brief The operations loop. This is called by the main loop and during
 * playsheet execution. This allows all of the subsystems to run.
 */
void operationsLoop() {
    ESPCrashMonitor.iAmAlive();
    Console.checkInterrupt();
    taskMan.execute();
    #ifdef ENABLE_MDNS
        mdns.update();
    #endif
    #ifdef ENABLE_OTA
        ArduinoOTA.handle();
    #endif
    mqttClient.loop();
}

/**
 * @brief Setup routine. This is called *once* at startup and is essentially
 * the boot sequence. This initializes all subsystems.
 */
void setup() {
    initSerial();
    initCrashMonitor();
    initOutputs();
    #ifndef MODEL_1
    initComBus();
    #endif
    initRelayModules();
    initFilesystem();
    initWiFi();
    initMDNS();
    initOTA();
    initMQTT();
    initTaskManager();
    initConsole();
    Serial.println(F("INFO: Boot sequence complete."));
    sysState = SystemState::NORMAL;
    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

/**
 * @brief The main program loop. Runs the operations loop and also runs the
 * play sheet sequence if enabled.
 */
void loop() {
    operationsLoop();
    if (!terminateSequence) {
        playSequenceSheet();
    }
}