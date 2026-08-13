#ifdef DEVICE_NEWSHEEN

#include "Boards/Newsheen/NewsheenInput.h"
#include "Data/InputKeys.h"

NewsheenInput::NewsheenInput()
    : lastPin(-1), lastEventMs(0) {
    pinMode(NEWSHEEN_BTN_USER, INPUT_PULLUP);
    pinMode(NEWSHEEN_BTN_BOOT, INPUT_PULLUP);
}

// User button wins over BOOT so a confirm always beats a stray "next".
int NewsheenInput::pressedPin() {
    if (digitalRead(NEWSHEEN_BTN_USER) == LOW) return NEWSHEEN_BTN_USER;
    if (digitalRead(NEWSHEEN_BTN_BOOT) == LOW) return NEWSHEEN_BTN_BOOT;
    return -1;
}

char NewsheenInput::scan() {
    int pin = pressedPin();

    // Nothing held: re-arm once the bus has been idle long enough to debounce.
    if (pin == -1) {
        if (lastPin != -1 && (millis() - lastEventMs) >= DEBOUNCE_MS) {
            lastPin = -1;
        }
        return KEY_NONE;
    }

    // Same button still held -> already reported (no auto-repeat).
    if (pin == lastPin) {
        return KEY_NONE;
    }

    // New press.
    lastPin = pin;
    lastEventMs = millis();

    switch (pin) {
        case NEWSHEEN_BTN_USER: return KEY_OK;
        case NEWSHEEN_BTN_BOOT: return KEY_ARROW_RIGHT;
        default:                return KEY_NONE;
    }
}

char NewsheenInput::readChar() {
    return scan();
}

char NewsheenInput::handler() {
    while (true) {
        char c = scan();
        if (c != KEY_NONE) return c;
        delay(5);
    }
}

void NewsheenInput::waitPress(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (true) {
        if (scan() != KEY_NONE) return;
        if (timeoutMs > 0 && (millis() - start) >= timeoutMs) return;
        delay(5);
    }
}

#endif
