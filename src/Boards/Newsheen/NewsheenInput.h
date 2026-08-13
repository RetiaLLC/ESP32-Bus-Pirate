#pragma once

#ifdef DEVICE_NEWSHEEN

#include "Interfaces/IInput.h"
#include <Arduino.h>

// Newsheen (Pusheen puck) buttons - active-low with internal pull-ups.
#define NEWSHEEN_BTN_USER  17   // SW3 user button
#define NEWSHEEN_BTN_BOOT  0    // BOOT strapping button (SW2)

// The Bit Pirate command shell runs over USB-CDC / Wi-Fi; the puck has no screen,
// so these two buttons only drive the boot-time terminal picker
// (HorizontalSelector::selectHeadless):
//
//   USER (SW3) -> KEY_OK          accept the highlighted terminal type
//   BOOT (SW2) -> KEY_ARROW_RIGHT cycle to the next terminal type
//
// With no press the picker times out to USB Serial - the puck's primary link.
class NewsheenInput final : public IInput {
public:
    NewsheenInput();

    char handler() override;
    char readChar() override;
    void waitPress(uint32_t timeoutMs) override;

private:
    char scan();          // edge-detected, debounced single keypress (KEY_NONE if idle)
    int  pressedPin();    // currently-held pin, or -1

    int      lastPin;
    uint32_t lastEventMs;
    static constexpr uint32_t DEBOUNCE_MS = 30;
};

#endif
