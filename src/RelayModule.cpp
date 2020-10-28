#include "RelayModule.h"

RelayModule::RelayModule(Adafruit_MCP23017 *busController, RelayModulePort port) {
	this->_busController = busController;
	this->_port = port;
}

uint8_t RelayModule::getRelayAddress(RelaySelect relay) {
	uint8_t result = -1;
	switch (this->_port)
	{
	case RelayModulePort::PORTA:
		switch (relay) {
			case RelaySelect::RELAY1:
				result = PORTA_RELAY_1;
				break;
			case RelaySelect::RELAY2:
				result = PORTA_RELAY_2;
				break;
			case RelaySelect::RELAY3:
				result = PORTA_RELAY_3;
				break;
			case RelaySelect::RELAY4:
				result = PORTA_RELAY_4;
				break;
			case RelaySelect::RELAY5:
				result = PORTA_RELAY_5;
				break;
			case RelaySelect::RELAY6:
				result = PORTA_RELAY_6;
				break;
			case RelaySelect::RELAY7:
				result = PORTA_RELAY_7;
				break;
			case RelaySelect::RELAY8:
				result = PORTA_RELAY_8;
				break;
		}
		break;
	case RelayModulePort::PORTB:
		switch (relay)
		{
			case RelaySelect::RELAY1:
				result = PORTB_RELAY_1;
				break;
			case RelaySelect::RELAY2:
				result = PORTB_RELAY_2;
				break;
			case RelaySelect::RELAY3:
				result = PORTB_RELAY_3;
				break;
			case RelaySelect::RELAY4:
				result = PORTB_RELAY_4;
				break;
			case RelaySelect::RELAY5:
				result = PORTB_RELAY_5;
				break;
			case RelaySelect::RELAY6:
				result = PORTB_RELAY_6;
				break;
			case RelaySelect::RELAY7:
				result = PORTB_RELAY_7;
				break;
			case RelaySelect::RELAY8:
				result = PORTB_RELAY_8;
				break;
			default:
				break;
		}
		break;
	default:
		break;
	}

	return result;
}

void RelayModule::init() {
	for (uint8_t i = 1; i <= 8; i++) {
		uint8_t address = this->getRelayAddress((RelaySelect)i);
		this->_busController->pinMode(address, OUTPUT);
		this->_busController->digitalWrite(address, LOW);
	}
}

ModuleRelayState RelayModule::getState(RelaySelect relay) {
	uint8_t address = this->getRelayAddress(relay);
	uint8_t state = this->_busController->digitalRead(address);
	return (ModuleRelayState)state;
}

void RelayModule::setState(RelaySelect relay, ModuleRelayState state) {
	if (this->getState(relay) != state) {
		uint8_t address = this->getRelayAddress(relay);
		this->_busController->digitalWrite(address, (uint8_t)state);
	}
}

bool RelayModule::isOpen(RelaySelect relay) {
	return this->getState(relay) == ModuleRelayState::OPEN;
}

bool RelayModule::isClosed(RelaySelect relay) {
	return this->getState(relay) == ModuleRelayState::CLOSED;
}

void RelayModule::close(RelaySelect relay) {
	this->setState(relay, ModuleRelayState::CLOSED);
}

void RelayModule::open(RelaySelect relay) {
	this->setState(relay, ModuleRelayState::OPEN);
}

void RelayModule::allRelaysClosed() {
	for (uint8_t i = 1; i <= 8; i++) {
		this->close((RelaySelect)i);
	}
}

void RelayModule::allRelaysOpen() {
	for (uint8_t i = 1; i <= 8; i++) {
		this->open((RelaySelect)i);
	}
}

