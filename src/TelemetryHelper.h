#ifndef TELEMETRYHELPER_H
#define TELEMETRYHELPER_H

#include <Arduino.h>

/**
 * Possible system states.
 */
enum class SystemState: uint8_t {
    /**
     * System is currently running the boot sequence.
     */
    BOOTING = 0,

    /**
     * System is operating normally.
     */
    NORMAL = 1,

    /**
     * System is currently performing an OTA update.
     */
    UPDATING = 2,

    /**
     * System is disabled. Either intentionally by the user or
     * by the system due to some internal system condition such
     * as falling into failsafe/configuration mode.
     */
    DISABLED = 3
};

enum class ControlCommand: uint8_t {
    DISABLE = 0,
    ENABLE = 1,
    REBOOT = 2,
    REQUEST_STATUS = 3,
    PAUSE_SEQUENCE = 4,
    PLAY_SEQUENCE = 5
};

/**
 * Helper class providing static telemetry helper methods.
 */
class TelemetryHelper
{
public:
    /**
     * Gets a string describing the specified MQTT client state.
     * @param state The state to get the description for.
     * @return A string describing the MQTT state.
     */
    static String getMqttStateDesc(int state);
};

#endif