#ifdef DEVICE_NEWSHEEN

#include "Boards/Newsheen/NewsheenBoard.h"
#include <Arduino.h>

// Seeed Wio-SX1262 RX-enable / RF switch. The SX1262 drives its own TX/RX antenna
// switch through DIO2 (LoRaService sets USE_DIO2_ANT_SWITCH), while RXEN gates the
// RX LNA path. Park it HIGH so the radio can receive; TX is unaffected. LoRaService
// leaves RADIO_RXEN at -1, so nothing else touches this pin.
static constexpr int8_t NEWSHEEN_LORA_RXEN = 14;

void NewsheenBoard::initialize() {
    // Enable the LoRa RX path (DIO2 still owns the TX/RX antenna switch).
    pinMode(NEWSHEEN_LORA_RXEN, OUTPUT);
    digitalWrite(NEWSHEEN_LORA_RXEN, HIGH);

    // NOTE: no boot NeoPixel pulse here. FastLED owns the WS2812 ring on GPIO16
    // via LedService (the `led` command); installing a second FastLED controller
    // on GPIO16 at boot leaves the S3 RMT channel allocated, and LedService's
    // `FastLED = CFastLED()` re-init on the same pin then resets the chip when the
    // user enters LED mode. The ring is driven exclusively through `led`.

    deviceView.initialize();
}

IDeviceView& NewsheenBoard::getDeviceView() {
    return deviceView;
}

IInput& NewsheenBoard::getDeviceInput() {
    return deviceInput;
}

IHostSerial& NewsheenBoard::getHostSerial() {
    return hostSerial;
}

#endif
