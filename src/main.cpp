/**
 * CLController v1.0
 * Author:
 *  Cyrus Brunner
 * 
 * I/O Expansion:
 * 
 * The main controller board has an MCP23017 which provides 16 digital
 * GPIOs and is located at PRIMARY_I2C_ADDRESS. This is what controls the 8 outputs
 * on each of the 2 ports on the main board.  However, it is possible to expand
 * this even further by adding additional MCP23017s on the I2C bus.  As such, the
 * main board provides a 2-pin I2C header that additional expanders to connect to.
 */

// TODO Possible design changes:
// 1) Change ESP8266 to ESP32
// 2) Split up into further modules - 1] Main controller board. 2] I/O module (MCP3017). 3] Relay modules (include ULN2803)
// 3) Create abstraction library for relay module (it's essentially an 8bit interface).
// 4) Come up with a way to detect relay module presence.

#include <Arduino.h>
#include <FS.h>
#include <time.h>
#include <Wire.h>
#include <vector>
#include "Adafruit_MCP23017.h"
#include "ArduinoJson.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "Console.h"
#include "LED.h"
#include "PubSubClient.h"
#include "ResetManager.h"
#include "TaskScheduler.h"
#include "TelemetryHelper.h"
#include "RelayModule.h"
#include "Playsheet.h"
#include "config.h"         // This should be included last.

#define FIRMWARE_VERSION "1.0"

#define PRIMARY_I2C_ADDRESS 0x20

#define PIN_WIFI_LED 16
#define PIN_RUN_LED 14

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
bool primaryExpanderFound = false;
std::vector<byte> devicesFound;
std::vector<reference_wrapper<Adafruit_MCP23017>> additionalBusses;
std::vector<reference_wrapper<RelayModule>> relayModules;
Adafruit_MCP23017 primaryBus;
LED wifiLED(PIN_WIFI_LED, NULL);
LED runLED(PIN_RUN_LED, NULL);
config_t config;
bool filesystemMounted = false;
volatile bool terminateSequence = false;
volatile SystemState sysState = SystemState::BOOTING;

void addBus(Adafruit_MCP23017 &bus) {
    additionalBusses.push_back(std::ref(bus));
}

Adafruit_MCP23017 getBus(std::size_t index) {
    if (index < 0 || index > additionalBusses.size() - 1) {
        index = 0;
    }

    return additionalBusses.at(index).get();
}

void addModule(RelayModule &module) {
    relayModules.push_back(std::ref(module));
}

RelayModule getModule(std::size_t index) {
    if (index < 0 || index > relayModules.size() - 1) {
        index = 0;
    }

    return relayModules.at(index).get();
}

/**
 * Synchronize the local system clock via NTP. Note: This does not take DST
 * into account. Currently, you will have to adjust the CLOCK_TIMEZONE define
 * manually to account for DST when needed.
 */
void onSyncClock() {
    configTime(config.clockTimezone * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("INIT: Waiting for NTP time sync...");
    delay(500);
    while (!time(nullptr)) {
        ESPCrashMonitor.iAmAlive();
        Serial.print(F("."));
        delay(500);
    }

    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    
    Serial.println(F(" DONE"));
    Serial.print(F("INFO: Current time: "));
    Serial.println(asctime(timeinfo));
}

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
 * Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
    wifiLED.off();
    runLED.off();
    sysState = SystemState::NORMAL;
    terminateSequence = false;
    publishSystemState();
}

/**
 * Prints network information details to the serial console.
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
    WiFi.printDiag(Serial);
}

/**
 * Scan for available networks and dump each discovered network to the console.
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

void reboot() {
    Serial.println(F("INFO: Rebooting..."));
    Serial.flush();
    delay(1000);
    ResetManager.softReset();
}

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
    doc["mqttControlChannel"] = config.mqttTopicControl;
    doc["mqttStatusChannel"] = config.mqttTopicStatus;
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

void printWarningAndContinue(const __FlashStringHelper *message) {
    Serial.println();
    Serial.println(message);
    Serial.print(F("INFO: Continuing... "));
}

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
    config.mqttTopicControl = doc.containsKey("mqttControlChannel") ? doc["mqttControlChannel"].as<String>() : MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = doc.containsKey("mqttStatusChannel") ? doc["mqttStatusChannel"].as<String>() : MQTT_TOPIC_STATUS;
    config.mqttUsername = doc.containsKey("mqttUsername") ? doc["mqttUsername"].as<String>() : "";
    config.mqttPassword = doc.containsKey("mqttPassword") ? doc["mqttPassword"].as<String>() : "";

    #ifdef ENABLE_OTA
        config.otaPort = doc.containsKey("otaPort") ? doc["otaPort"].as<uint16_t>() : MQTT_PORT;
        config.otaPassword = doc.containsKey("otaPassword") ? doc["otaPassword"].as<String>() : OTA_PASSWORD;
    #endif

    doc.clear();
    Serial.println(F("DONE"));
}

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
    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
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
    
    uint8_t modIdx = 0;
    ModuleRelayState newState = ModuleRelayState::OPEN;
    RelaySelect relay = RelaySelect::RELAY1;

    JsonArray sequences = doc["seq"].as<JsonArray>();
    for (auto sequence : sequences) {
        Sequence seq;
        JsonArray states = sequence["states"];
        for (auto state : states) {
            modIdx = state["modIdx"].as<uint8_t>();
            if (strcmp(state["lsState"].as<const char*>(), "on") == 0) {
                newState = ModuleRelayState::CLOSED;
            }
            else {
                newState = ModuleRelayState::OPEN;
            }

            relay = (RelaySelect)state["lsIdx"].as<uint8_t>();
            SequenceState theState(modIdx, newState, relay);
            seq.states.push_back(theState);
        }

        seq.delay = sequence["delayMs"].as<unsigned long>();
        Playsheet.sequences.push_back(seq);
    }

    doc.clear();
    Serial.println(F("DONE"));
}

void playSequenceSheet() {
    if (terminateSequence) {
        return;
    }

    runLED.on();
    Serial.print(F("INFO: Executing sequence sheet: "));
    Serial.println(Playsheet.sheetName);

    size_t count = Playsheet.sequences.size();
    Serial.print(F("INFO: Sequence count: "));
    Serial.println(count);
    for (auto seq = Playsheet.sequences.begin(); seq != Playsheet.sequences.end(); seq++) {
        // This will introduce delays during sequences, but it beats blocking
        // other operations from processing and keeps the watchdog fed.
        // The ESP-32 would probably be better suited since we can run a
        // a second thread for running sequences and keep everything else
        // on the main loop.
        Serial.println(F("DEBUG: OpLoop"));
        operationsLoop();
        if (terminateSequence) {
            break;
        }

        Serial.print(F("DEBUG: seq delay = "));
        Serial.print(seq->delay);
        Serial.print(F(", state count = "));
        Serial.println(seq->states.size());
        for (auto state = seq->states.begin(); state != seq->states.end(); state++) {
            Serial.print(F("DEBUG: State modIdx = "));
            Serial.print(state->moduleIndex);
            Serial.print(F(", lsIdx = "));
            Serial.print((uint8_t)state->lightStringIndex);
            Serial.print(F(", lsState = "));
            Serial.println((uint8_t)state->lightStringState);
            getModule(state->moduleIndex).setState(state->lightStringIndex, state->lightStringState);
        }

        // delay for note duration + 30%
        delay(seq->delay * 1.30);
    }

    runLED.off();
    Serial.print(F("INFO: Finished running sequence: "));
    Serial.println(Playsheet.sheetName);
}

bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
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
            Serial.print(F("INFO: Subscribing to channel: "));
            Serial.println(config.mqttTopicControl);
            mqttClient.subscribe(config.mqttTopicControl.c_str());

            Serial.print(F("INFO: Publishing to channel: "));
            Serial.println(config.mqttTopicStatus);
        }
        else {
            String failReason = TelemetryHelper::getMqttStateDesc(mqttClient.state());
            Serial.print(F("ERROR: Failed to connect to MQTT broker: "));
            Serial.println(failReason);
            return false;
        }
    }

    return true;
}

void onCheckMqtt() {
    Serial.println(F("INFO: Checking MQTT connections status..."));
    if (reconnectMqttClient()) {
        Serial.println(F("INFO: Successfully reconnected to MQTT broker."));
        publishSystemState();
    }
    else {
        Serial.println(F("ERROR: MQTT connection lost and reconnect failed."));
        Serial.print(F("INFO: Retrying connection in "));
        Serial.print(CHECK_MQTT_INTERVAL % 1000);
        Serial.println(F(" seconds ..."));
    }
}

void allRelaysOn() {
    for (size_t i = 0; i < relayModules.size(); i++) {
        Serial.print(F("INFO: Turning all relays on module at address "));
        Serial.print(i);
        Serial.println(F(" on ..."));
        getModule(i).allRelaysClosed();
    }
}

void allRelaysOff() {
    for (size_t i = 0; i < relayModules.size(); i++) {
        Serial.print(F("INFO: Turning all relays on module at address "));
        Serial.print(i);
        Serial.println(F(" of ..."));
        getModule(i).allRelaysOpen();
    }
}

void handleControlRequest(const JsonObject &json) {
    if (json.containsKey("client_id")) {
        String id = json["client_id"].as<String>();
        id.toUpperCase();
        if (!id.equals(config.hostname)) {
            Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
            return;
        }
    }
    else {
        Serial.println(F("WARN: MQTT message does not contain client ID. Ignoring..."));
        return;
    }

    if (!json.containsKey("command")) {
        Serial.println(F("WARN: MQTT message does not contain a control command. Ignoring..."));
        return;
    }

    // When system is the "disabled" state, the only command it will accept
    // is "enable". All other commands are ignored.
    ControlCommand cmd = (ControlCommand)json["command"].as<uint8_t>();
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

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print(F("INFO: [MQTT] Message arrived: ["));
    Serial.print(topic);
    Serial.print(F("] "));
    Serial.println((char*)payload);

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, (char*)payload);
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

    doc.clear();
    JsonObject json = doc.as<JsonObject>();
    handleControlRequest(json);
}

/**
 * Enter fail-safe mode. This will suspend all tasks, disable relay activation,
 * and propmpt the user for configuration.
 */
void failSafe() {
    sysState = SystemState::DISABLED;
    terminateSequence = true;
    publishSystemState();
    ESPCrashMonitor.defer();
    Serial.println();
    Serial.println(F("ERROR: Entering failsafe (config) mode..."));
    taskMan.disableAll();
    runLED.off();
    wifiLED.on();
    Console.enterCommandInterpreter();
}

/**
 * Initializes the MDNS responder (if enabled).
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
 * Initialize the SPIFFS filesystem.
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
 * Initializes the MQTT client.
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
 * Attempt to connect to the configured WiFi network. This will break any existing connection first.
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

void scanBusDevices() {
    byte error;
    byte address;
    int devices = 0;

    Serial.print(F("DEBUG: SDA pin: "));
    Serial.println(SDA);
    Serial.print(F("DEBUG: SCL pin: "));
    Serial.println(SCL);
    
    Serial.println(F("INFO: Beginning I2C bus scan ..."));
    for (address = 1; address < 127; address++) {
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
            if (!primaryExpanderFound && address == PRIMARY_I2C_ADDRESS) {
                primaryExpanderFound = true;
            }
            else {
                devicesFound.push_back(address);
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

/**
 * Initializes the WiFi network interface.
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
 * Initializes the OTA update listener if enabled.
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
 * Callback routine for checking WiFi connectivity.
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

void initSerial() {
    Serial.begin(BAUD_RATE);
    Serial.setDebugOutput(true);
    Serial.println();
    Serial.print(F("CLController v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" booting ..."));
}

void initComBus() {
    Wire.begin();
    Serial.println(F("INIT: Initializing communication bus ..."));
    scanBusDevices();
    if (primaryExpanderFound) {
        Serial.println(F("INFO: Found primary host bus controller."));
        Serial.print(F("INFO: Secondary bus controllers found: "));
        Serial.println(devicesFound.size());
        
        primaryBus.begin(PRIMARY_I2C_ADDRESS);
        addBus(primaryBus);  // The primary bus controller should always be at index 0.

        for (std::size_t i = 0; i < devicesFound.size(); i++) {
            Adafruit_MCP23017 newBus;
            newBus.begin(devicesFound.at(i));
            addBus(newBus);
        }
    }
    else {
        terminateSequence = true;
        Serial.println(F("ERROR: Primary host bus controller not found!!"));
        // TODO HCF???
    }

    Serial.println(F("INIT: Comm bus initialization complete."));
}

void initRelayModules() {
    Serial.println(F("INIT: Initializing relay modules ..."));
    if (!primaryExpanderFound) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: No bus controller found."));
        return;
    }

    Serial.print(F("INIT: Init relay modules A and B at bus address: "));
    Serial.println(PRIMARY_I2C_ADDRESS, HEX);
    RelayModule moduleA(&primaryBus, RelayModulePort::PORTA);
    moduleA.init();
    addModule(moduleA);

    RelayModule moduleB(&primaryBus, RelayModulePort::PORTB);
    moduleB.init();
    addModule(moduleB);

    // Skip ahead by 1 since the first bus is always the primary.
    for (size_t i = 1; i < additionalBusses.size(); i++) {
        Serial.print(F("INIT: Init relay modules A and B at bus address: "));
        Serial.println(devicesFound.at(i), HEX);
        Adafruit_MCP23017 bus = getBus(i);

        RelayModule newModuleA(&bus, RelayModulePort::PORTA);
        newModuleA.init();
        addModule(newModuleA);

        RelayModule newModuleB(&bus, RelayModulePort::PORTB);
        newModuleB.init();
        addModule(newModuleB);
    }

    Serial.println(F("INIT: Relay module initialization complete."));
}

void initOutputs() {
    Serial.print(F("INIT: Initializing outputs ..."));
    wifiLED.init();
    wifiLED.on();
    runLED.init();
    runLED.on();
    Serial.println(F("DONE"));
}

void handleRunPlaysheet() {
    terminateSequence = false;
}

void handleInterruptPlaysheet() {
    Serial.println(F("INFO: Playsheet interrupt..."));
    terminateSequence = true;
}

void handleNewHostname(const char* newHostname) {
    config.hostname = newHostname;
    initMDNS();
}

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

void handleSwitchToStatic(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns) {
    config.ip = newIp;
    config.sm = newSm;
    config.gw = newGw;
    config.dns = newDns;
    Serial.println(F("INFO: Set static network config."));
    WiFi.config(config.ip, config.gw, config.sm, config.dns);
}

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

void handleWifiConfig(String newSsid, String newPassword) {
    config.ssid = newSsid;
    config.password = newPassword;
    connectWifi();
}

void handleSaveConfig() {
    saveConfiguration();
    WiFi.disconnect(true);
    onCheckWifi();
}

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

    Serial.println(F("DONE"));
}

/**
 * Initializes the task manager and all recurring tasks.
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
 * Initializes the crash monitor and dump any previous crash data to the serial console.
 */
void initCrashMonitor() {
    Serial.print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    Serial.println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
}

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

void setup() {
    initSerial();
    initCrashMonitor();
    initOutputs();
    initComBus();
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

void loop() {
    operationsLoop();
    if (!terminateSequence) {
        playSequenceSheet();
    }
}