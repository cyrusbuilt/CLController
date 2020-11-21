#ifndef _PLAYSHEET_H
#define _PLAYSHEET_H

#include <functional>
#include <vector>
#include "RelayModule.h"

using namespace std;

class SequenceState {
public:
    SequenceState(uint8_t modIdx, ModuleRelayState lsState, RelaySelect lsIndex) {
        moduleIndex = modIdx;
        lightStringState = lsState;
        lightStringIndex = lsIndex;
    };

    SequenceState() {
        moduleIndex = 0;
        lightStringState = ModuleRelayState::OPEN;
        lightStringIndex = RelaySelect::RELAY1;
    }

    uint8_t moduleIndex;
    ModuleRelayState lightStringState;
    RelaySelect lightStringIndex;
};

class Sequence {
public:
    Sequence() { delay = 0; }
    unsigned long delay;
    std::vector<SequenceState> states;
};

class PlaysheetClass {
public:
    const char* sheetName;
    std::vector<Sequence> sequences;
};

extern PlaysheetClass Playsheet;

#endif