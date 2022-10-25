#ifndef _RELAY_MODULE_H
#define _RELAY_MODULE_H

#include <Arduino.h>
#include "Adafruit_MCP23017.h"
#include "RelayMap.h"

/**
 * @brief Possible port addresses for relay modules.
 */
enum class RelayModulePort : uint8_t {
	/**
	 * @brief Relay module port A.
	 */
	PORTA = 0,

	/**
	 * @brief Relay module port B.
	 */
	PORTB = 1
};

/**
 * @brief Possible relay addresses on a controller (port-independent).
 */
enum class RelaySelect : uint8_t {
	/**
	 * @brief Relay 1.
	 */
	RELAY1 = 1,

	/**
	 * @brief Relay 2.
	 */
	RELAY2 = 2,

	/**
	 * @brief Relay 3.
	 */
	RELAY3 = 3,

	/**
	 * @brief Relay 4.
	 */
	RELAY4 = 4,

	/**
	 * @brief Relay 5.
	 */
	RELAY5 = 5,

	/**
	 * @brief Relay 6.
	 */
	RELAY6 = 6,

	/**
	 * @brief Relay 7.
	 */
	RELAY7 = 7,

	/**
	 * @brief Relay 8.
	 */
	RELAY8 = 8
};

/**
 * @brief Possible relay states.
 */
enum class ModuleRelayState : uint8_t {
	/**
	 * @brief The relay is open (coil off).
	 */
	OPEN = LOW,

	/**
	 * @brief The relay is closed (coil on).
	 */
	CLOSED = HIGH
};

/**
 * @brief Driver class for relay modules.
 */
class RelayModule
{
public:
	/**
	 * @brief Construct a new Relay Module object.
	 * 
	 * @param busController The bus controller the module is attached to.
	 * @param port The module port it is attached to.
	 */
	RelayModule(Adafruit_MCP23017 *busController, RelayModulePort port);

	/**
	 * @brief Detects whether or not the relay module is connected.
	 * 
	 * @return true True if the module is detected.
	 * @return false False if the module cannot be found.
	 */
	bool detect();

	/**
	 * @brief Initializes the relay module.
	 */
	void init();

	/**
	 * @brief Set the state of a relay on the module.
	 * 
	 * @param relay The relay to change the state of.
	 * @param state The state to set.
	 */
	void setState(RelaySelect relay, ModuleRelayState state);

	/**
	 * @brief Get the state of the specified relay.
	 * 
	 * @param relay The relay to get the state of.
	 * @return ModuleRelayState The state of the relay.
	 */
	ModuleRelayState getState(RelaySelect relay);

	/**
	 * @brief Checks to see if the specified relay is open.
	 * 
	 * @param relay The relay to check.
	 * @return true if the relay is open.
	 * @return false if the relay is closed.
	 */
	bool isOpen(RelaySelect relay);

	/**
	 * @brief Checks to see if the specified relay is closed.
	 * 
	 * @param relay The relay to check.
	 * @return true if the relay is closed.
	 * @return false if the relay is open.
	 */
	bool isClosed(RelaySelect relay);

	/**
	 * @brief Closes the specified relay.
	 * 
	 * @param relay The relay to close.
	 */
	void close(RelaySelect relay);

	/**
	 * @brief Opens the specified relay.
	 * 
	 * @param relay The relay to open.
	 */
	void open(RelaySelect relay);

	/**
	 * @brief Closes all relays.
	 */
	void allRelaysClosed();

	/**
	 * @brief Opens all relays.
	 * 
	 */
	void allRelaysOpen();
	
private:
	uint8_t getRelayAddress(RelaySelect relay);
	RelayModulePort _port;
	Adafruit_MCP23017 *_busController;
};

#endif