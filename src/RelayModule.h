#ifndef _RELAY_MODULE_H
#define _RELAY_MODULE_H

#include <Arduino.h>
#include "Adafruit_MCP23017.h"
#include "RelayMap.h"

enum class RelayModulePort : uint8_t {
	PORTA = 0,
	PORTB = 1
};

enum class RelaySelect : uint8_t {
	RELAY1 = 1,
	RELAY2 = 2,
	RELAY3 = 3,
	RELAY4 = 4,
	RELAY5 = 5,
	RELAY6 = 6,
	RELAY7 = 7,
	RELAY8 = 8
};

enum class ModuleRelayState : uint8_t {
	OPEN = LOW,
	CLOSED = HIGH
};

class RelayModule
{
public:
	RelayModule(Adafruit_MCP23017 *busController, RelayModulePort port);
	void init();
	void setState(RelaySelect relay, ModuleRelayState state);
	ModuleRelayState getState(RelaySelect relay);
	bool isOpen(RelaySelect relay);
	bool isClosed(RelaySelect relay);
	void close(RelaySelect relay);
	void open(RelaySelect relay);
	void allRelaysClosed();
	void allRelaysOpen();
private:
	uint8_t getRelayAddress(RelaySelect relay);
	RelayModulePort _port;
	Adafruit_MCP23017 *_busController;
};

#endif