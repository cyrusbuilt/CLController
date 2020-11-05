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
String hostname = DEVICE_NAME;
String ssid = DEFAULT_SSID;
String password = DEFAULT_PASSWORD;
String mqttBroker = MQTT_BROKER;
String controlChannel = MQTT_TOPIC_CONTROL;
String statusChannel = MQTT_TOPIC_STATUS;
String mqttUsername = "";
String mqttPassword = "";
int mqttPort = MQTT_PORT;
bool isDHCP = false;
bool filesystemMounted = false;
volatile bool terminateSequence = false;
volatile SystemState sysState = SystemState::BOOTING;
#ifdef ENABLE_OTA
    int otaPort = OTA_HOST_PORT;
    String otaPassword = OTA_PASSWORD;
#endif

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
    configTime(CLOCK_TIMEZONE * 3600, 0, "pool.ntp.org", "time.nist.gov");

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
        doc["client_id"] = hostname;
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;
        doc["playingSequence"] = !terminateSequence;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(statusChannel.c_str(), jsonStr.c_str(), len)) {
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
    doc["hostname"] = hostname;
    doc["useDhcp"] = isDHCP;
    doc["ip"] = ip.toString();
    doc["gateway"] = gw.toString();
    doc["subnetmask"] = sm.toString();
    doc["dnsServer"] = dns.toString();
    doc["wifiSSID"] = ssid;
    doc["wifiPassword"] = password;
    doc["mqttBroker"] = mqttBroker;
    doc["mqttPort"] = mqttPort;
    doc["mqttControlChannel"] = controlChannel;
    doc["mqttStatusChannel"] = statusChannel;
    doc["mqttUsername"] = mqttUsername;
    doc["mqttPassword"] = mqttPassword;
    #ifdef ENABLE_OTA
        doc["otaPort"] = otaPort;
        doc["otaPassword"] = otaPassword;
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

void loadConfiguration() {
    Serial.print(F("INFO: Loading config file "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.println(F(" ... "));
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
    if (size > 1024) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Config file size is too large. Using default config."));
        configFile.close();
        return;
    }

    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);
    configFile.close();

    StaticJsonDocument<350> doc;
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Fail to parse config file to JSON. Using default config."));
        buf.release();
        return;
    }

    hostname = doc["hostname"].as<String>();
    isDHCP = doc["isDhcp"].as<bool>();
    if (!ip.fromString(doc["ip"].as<String>())) {
        printWarningAndContinue(F("WARN: Invalid IP in configuration. Falling back to factory default."));
    }

    if (!gw.fromString(doc["gateway"].as<String>())) {
        printWarningAndContinue(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
    }

    if (!sm.fromString(doc["subnetmask"].as<String>())) {
        printWarningAndContinue(F("WARN: Invalid subnet mask in configuration. Falling back to factory default."));
    }

    if (!dns.fromString(doc["dns"].as<String>())) {
        printWarningAndContinue(F("WARN: Invalid DNS IP in configuration. Falling back to factory default."));
    }

    ssid = doc["wifiSSID"].as<String>();
    password = doc["wifiPassword"].as<String>();
    mqttBroker = doc["mqttBroker"].as<String>();
    mqttPort = doc["mqttPort"].as<int>();
    controlChannel = doc["mqttControlChannel"].as<String>();
    statusChannel = doc["mqttStatusChannel"].as<String>();
    mqttUsername = doc["mqttUsername"].as<String>();
    mqttPassword = doc["mqttPassword"].as<String>();
    #ifdef ENABLE_OTA
        otaPort = doc["otaPort"].as<int>();
        otaPassword = doc["otaPassword"].as<String>();
    #endif

    doc.clear();
    buf.release();
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

void playSequenceSheet() {
    if (terminateSequence) {
        return;
    }

    Serial.print(F("INFO: Loading sequence file: "));
    Serial.print(PLAYSHEET_FILE_PATH);
    Serial.println(F(" ..."));

    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    if (!SPIFFS.exists(PLAYSHEET_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Sequence file does not exist."));
        return;
    }

    File sequenceFile = SPIFFS.open(PLAYSHEET_FILE_PATH, "r");
    if (!sequenceFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to open sequence file."));
        return;
    }

    size_t size = sequenceFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    sequenceFile.readBytes(buf.get(), size);
    sequenceFile.close();

    DynamicJsonDocument doc(size);
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to parse sequence file to JSON."));
        buf.release();
        return;
    }

    buf.release();
    Serial.println(F("DONE"));

    runLED.on();
    String sheetName = doc["name"].as<String>();
    Serial.print(F("INFO: Executing sequence sheet: "));
    Serial.println(sheetName);
    JsonArray sequences = doc["seq"].as<JsonArray>();
    for (auto sequence : sequences) {
        // This will introduce delays during sequences, but it beats blocking
        // other operations from processing and keeps the watchdog fed.
        // The ESP-32 would probably be better suited since we can run a
        // a second thread for running sequences and keep everything else
        // on the main loop.
        operationsLoop();
        if (terminateSequence) {
            break;
        }

        JsonArray states = sequence["states"];
        for (auto state : states) {
            RelayModule module = getModule(state["modIdx"].as<uint8_t>());

            String stateStr = state["lsState"].as<String>();
            stateStr.trim();
            stateStr.toLowerCase();

            ModuleRelayState newState = "on" ? ModuleRelayState::CLOSED : ModuleRelayState::OPEN;
            RelaySelect relay = (RelaySelect)state["lsIdx"].as<uint8_t>();
            module.setState(relay, newState);
        }

        delay(sequence["delayMs"].as<unsigned long>());
    }

    Serial.print(F("INFO: Finished running sequence: "));
    Serial.println(sheetName);
    doc.clear();
    runLED.off();
}

bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
        Serial.print(F("INFO: Attempting to establish MQTT connection to "));
        Serial.print(mqttBroker);
        Serial.print(F(" on port "));
        Serial.print(mqttPort);
        Serial.println(F(" ... "));

        bool didConnect = false;
        if (mqttUsername.length() > 0 && mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(hostname.c_str(), mqttUsername.c_str(), mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(hostname.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to channel: "));
            Serial.println(controlChannel);
            mqttClient.subscribe(controlChannel.c_str());

            Serial.print(F("INFO: Publishing to channel: "));
            Serial.println(statusChannel);
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

void handleControlRequest(String id, ControlCommand cmd) {
    id.toUpperCase();
    if (!id.equals(hostname)) {
        Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
        return;
    }

    // When system is the "disabled" state, the only command it will accept
    // is "enable". All other commands are ignored.
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

    // It's a lot easier to deal with if we just convert the payload
    // to a string first.
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.println(msg);

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

    String id = doc["client_id"].as<String>();
    ControlCommand cmd = (ControlCommand)doc["command"].as<uint8_t>();
    doc.clear();
    handleControlRequest(id, cmd);
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
            if (!mdns.begin(hostname)) {
                Serial.println(F(" FAILED"));
                return;
            }

            #ifdef ENABLE_OTA
                mdns.addService(hostname, "ota", otaPort);
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
    loadConfiguration();
}

/**
 * Initializes the MQTT client.
 */
void initMQTT() {
    Serial.print(F("INIT: Initializing MQTT client... "));
    mqttClient.setServer(mqttBroker.c_str(), mqttPort);
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
    Serial.println(F("DEBUG: Setting mode..."));
    WiFi.mode(WIFI_STA);
    Serial.println(F("DEBUG: Disconnect and clear to prevent auto connect..."));
    WiFi.persistent(false);
    WiFi.disconnect(true);
    ESPCrashMonitor.defer();

    delay(1000);
    if (isDHCP) {
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        WiFi.config(ip, gw, sm, gw);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(ssid, password);
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
    
    Serial.println(F("INFO: Beginning I2C bus scan ..."));
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
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

        devices++;
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
    Serial.print(ssid);
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
            ArduinoOTA.setPort(otaPort);
            ArduinoOTA.setHostname(hostname.c_str());
            ArduinoOTA.setPassword(otaPassword.c_str());
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
        Serial.println(F("ERROR: Primary host bus controller not found!!"));
        // TODO HCF???
    }

    Serial.println(F("INIT: Comm bus initialization complete."));
}

void initRelayModules() {
    Serial.println(F("INIT: Initializing relay modules ..."));
    
    Serial.print(F("INIT: Init relay modules A and B at bus address: "));
    Serial.println(PRIMARY_I2C_ADDRESS, HEX);
    RelayModule moduleA(&primaryBus, RelayModulePort::PORTA);
    moduleA.init();
    addModule(moduleA);

    RelayModule moduleB(&primaryBus, RelayModulePort::PORTB);
    moduleB.init();
    addModule(moduleB);

    for (size_t i = 0; i < additionalBusses.size(); i++) {
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
    hostname = newHostname;
    initMDNS();
}

void handleSwitchToDhcp() {
    if (isDHCP) {
        Serial.println(F("INFO: DHCP mode already set. Skipping..."));
        Serial.println();
    }
    else {
        isDHCP = true;
        Serial.println(F("INFO: Set DHCP mode."));
        WiFi.config(0U, 0U, 0U, 0U);
    }
}

void handleSwitchToStatic(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns) {
    ip = newIp;
    sm = newSm;
    gw = newGw;
    dns = newDns;
    Serial.println(F("INFO: Set static network config."));
    WiFi.config(ip, gw, sm, dns);
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
    ssid = newSsid;
    password = newPassword;
    connectWifi();
}

void handleSaveConfig() {
    saveConfiguration();
    WiFi.disconnect(true);
    onCheckWifi();
}

void handleMqttConfigCommand(String newBroker, int newPort, String newUsername, String newPassw, String newConChan, String newStatChan) {
    mqttClient.unsubscribe(controlChannel.c_str());
    mqttClient.disconnect();

    mqttBroker = newBroker;
    mqttPort = newPort;
    mqttUsername = newUsername;
    mqttPassword = newPassw;
    controlChannel = newConChan;
    statusChannel = newStatChan;

    initMQTT();
    Serial.println();
}

void initConsole() {
    Serial.print(F("INIT: Initializing console... "));

    Console.setHostname(hostname);
    Console.setMqttConfig(mqttBroker, mqttPort, mqttUsername, mqttPassword, controlChannel, statusChannel);
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