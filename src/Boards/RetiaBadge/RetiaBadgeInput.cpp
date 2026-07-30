#ifdef DEVICE_RETIA_BADGE

#include "Boards/RetiaBadge/RetiaBadgeInput.h"
#include "Data/InputKeys.h"

RetiaBadgeInput::RetiaBadgeInput()
    : lastPin(-1), lastEventMs(0) {
    pinMode(RETIA_BTN_LEFT,  INPUT_PULLUP);
    pinMode(RETIA_BTN_UP,    INPUT_PULLUP);
    pinMode(RETIA_BTN_DOWN,  INPUT_PULLUP);
    pinMode(RETIA_BTN_RIGHT, INPUT_PULLUP);
    pinMode(RETIA_BTN_B,     INPUT_PULLUP);
    pinMode(RETIA_BTN_A,     INPUT_PULLUP);
    pinMode(RETIA_BTN_BOOT,  INPUT_PULLUP);
}

// Highest priority first so that a confirm (A) always wins over a stray direction.
int RetiaBadgeInput::pressedPin() {
    if (digitalRead(RETIA_BTN_A)     == LOW) return RETIA_BTN_A;
    if (digitalRead(RETIA_BTN_B)     == LOW) return RETIA_BTN_B;
    if (digitalRead(RETIA_BTN_LEFT)  == LOW) return RETIA_BTN_LEFT;
    if (digitalRead(RETIA_BTN_RIGHT) == LOW) return RETIA_BTN_RIGHT;
    if (digitalRead(RETIA_BTN_UP)    == LOW) return RETIA_BTN_UP;
    if (digitalRead(RETIA_BTN_DOWN)  == LOW) return RETIA_BTN_DOWN;
    if (digitalRead(RETIA_BTN_BOOT)  == LOW) return RETIA_BTN_BOOT;
    return -1;
}

char RetiaBadgeInput::scan() {
    int pin = pressedPin();

    // Nothing held: re-arm once the bus has been idle long enough to debounce.
    if (pin == -1) {
        if (lastPin != -1 && (millis() - lastEventMs) >= DEBOUNCE_MS) {
            lastPin = -1;
        }
        return KEY_NONE;
    }

    // Same button still held -> already reported, emit nothing (no auto-repeat).
    if (pin == lastPin) {
        return KEY_NONE;
    }

    // New press.
    lastPin = pin;
    lastEventMs = millis();

    switch (pin) {
        case RETIA_BTN_LEFT:  return KEY_ARROW_LEFT;
        case RETIA_BTN_RIGHT: return KEY_ARROW_RIGHT;
        case RETIA_BTN_UP:    return KEY_ARROW_UP;
        case RETIA_BTN_DOWN:  return KEY_ARROW_DOWN;
        case RETIA_BTN_A:     return KEY_OK;
        case RETIA_BTN_BOOT:  return KEY_OK;
        case RETIA_BTN_B:     return KEY_DEL;
        default:              return KEY_NONE;
    }
}

char RetiaBadgeInput::readChar() {
    return scan();
}

char RetiaBadgeInput::handler() {
    while (true) {
        char c = scan();
        if (c != KEY_NONE) return c;
        delay(5);
    }
}

void RetiaBadgeInput::waitPress(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (true) {
        if (scan() != KEY_NONE) return;
        if (timeoutMs > 0 && (millis() - start) >= timeoutMs) return;
        delay(5);
    }
}

#endif
