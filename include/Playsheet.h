#ifndef _PLAYSHEET_H
#define _PLAYSHEET_H

#include <functional>
#include <vector>
#include "RelayModule.h"

using namespace std;

/**
 * @brief Represents an addressable string of lights.
 */
class LightString {
public:
    /**
     * @brief Default ctor.
     */
    LightString() {
        modIdx = 0;
        lsIdx = RelaySelect::RELAY1;
    }

    /**
     * @brief Construct a new LightString object with module index and light
     * string index.
     * 
     * @param modIdx The zero-based index of the relay module. 
     * @param lsIdx The 1-based index of the light string.
     */
    LightString(uint8_t modIdx, RelaySelect lsIdx) {
        this->modIdx = modIdx;
        this->lsIdx = lsIdx;
    }

    /**
     * @brief The zero-based index of the module the light string is attached to.
     */
    uint8_t modIdx;

    /**
     * @brief The one-based index of the light string.
     */
    RelaySelect lsIdx;
};

/**
 * @brief Represents a "note" to play. In this context, a "note" is defined
 * by one or more light strings along with the period the note should last,
 * and a letter representing the note (ie. 'C' or 'G'). The *special* note
 * 'R' indicates a rest period.
 */
class Note {
public:
    /**
     * @brief A letter representing the note.
     */
    String note;

    /**
     * @brief The note period.
     */
    int period;

    /**
     * @brief A vector of one or more light strings.
     */
    vector<LightString> lights;
};

/**
 * @brief Represents a "play sheet", which is a light sequence similar to a
 * sheet of music that defines the tempo, pause, beat, rest count, and a
 * collection of note definitions, along with a "melody" which defines the
 * order in which "notes" should be played. This abstraction essentially
 * treats the strings of lights as if they combined to form a musical
 * instrument that can be played. Instead of producing sound, the lights
 * flash in time with the melody of the song.
 */
class PlaysheetClass {
public:
    /**
     * @brief The name of the play sheet or "song".
     */
    const char* sheetName;

    /**
     * @brief The tempo in milliseconds.
     */
    int tempo;

    /**
     * @brief The pause in milliseconds.
     */
    int pause;

    /**
     * @brief The beat.
     */
    int beat;

    /**
     * @brief The rest count.
     */
    int restCount;

    /**
     * @brief A vector of "notes".
     */
    vector<Note> notes;

    /**
     * @brief The "melody" represented by a vector of chars, which defines
     * the order in which "notes" should be "played".
     */
    vector<String> melody;
};

/**
 * @brief Global playsheet instance.
 */
extern PlaysheetClass Playsheet;

#endif