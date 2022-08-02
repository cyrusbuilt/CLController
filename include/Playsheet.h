#ifndef _PLAYSHEET_H
#define _PLAYSHEET_H

#include <functional>
#include <vector>
#ifdef MODEL_1
#include "Relay.h"
#else
#include "RelayModule.h"
#endif

using namespace std;

class SequenceState {
public:
    #ifdef MODEL_1
    SequenceState(uint8_t modIdx, RelayState lsState, uint8_t lsIndex) {
        moduleIndex = modIdx;
        lightStringState = lsState;
        lightStringIndex = lsIndex;
    }
    #else
    SequenceState(uint8_t modIdx, ModuleRelayState lsState, RelaySelect lsIndex) {
        moduleIndex = modIdx;
        lightStringState = lsState;
        lightStringIndex = lsIndex;
    };
    #endif

    SequenceState() {
        moduleIndex = 0;
        #ifdef MODEL_1
        lightStringState = RelayOpen;
        lightStringIndex = 0;
        #else
        lightStringState = ModuleRelayState::OPEN;
        lightStringIndex = RelaySelect::RELAY1;
        #endif
    }

    uint8_t moduleIndex;
    #ifdef MODEL_1
    RelayState lightStringState;
    uint8_t lightStringIndex;
    #else
    ModuleRelayState lightStringState;
    RelaySelect lightStringIndex;
    #endif
};

class Sequence {
public:
    Sequence() { delay = 0; }
    unsigned long delay;
    vector<SequenceState> states;
};

class PlaysheetClass {
public:
    const char* sheetName;
    std::vector<Sequence> sequences;
};

extern PlaysheetClass Playsheet;

#endif