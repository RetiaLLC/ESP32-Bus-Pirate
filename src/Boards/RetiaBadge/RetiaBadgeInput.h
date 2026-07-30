#pragma once

#ifdef DEVICE_RETIA_BADGE

#include "Interfaces/IInput.h"
#include <Arduino.h>

// Retia 2024 DEF CON badge face buttons (active-low, external 10k pull-ups).
#define RETIA_BTN_LEFT   3
#define RETIA_BTN_UP     4
#define RETIA_BTN_DOWN   5
#define RETIA_BTN_RIGHT  6
#define RETIA_BTN_B      7
#define RETIA_BTN_A      8
#define RETIA_BTN_BOOT   0

// Physical d-pad + A/B navigation for the on-screen menus (HorizontalSelector,
// pinout views, adapter mode). The actual Bus Pirate command shell runs over the
// USB-CDC / Wi-Fi terminal; these buttons only drive the badge's own UI.
//
//   LEFT / RIGHT  -> move selection      (KEY_ARROW_LEFT / KEY_ARROW_RIGHT)
//   UP / DOWN     -> vertical menus       (KEY_ARROW_UP  / KEY_ARROW_DOWN)
//   A             -> confirm / enter       (KEY_OK)
//   B             -> back / delete          (KEY_DEL)
class RetiaBadgeInput final : public IInput {
public:
    RetiaBadgeInput();

    char handler() override;
    char readChar() override;
    void waitPress(uint32_t timeoutMs) override;

private:
    char scan();          // edge-detected, debounced single keypress (KEY_NONE if idle)
    int  pressedPin();    // lowest-priority-first currently-held pin, or -1

    int      lastPin;
    uint32_t lastEventMs;
    static constexpr uint32_t DEBOUNCE_MS = 30;
};

#endif
