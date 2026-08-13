#ifdef DEVICE_NEWSHEEN

#include "Boards/Newsheen/NewsheenBoard.h"
#include <Arduino.h>
#include <FastLED.h>

// Seeed Wio-SX1262 RX-enable / RF switch. The SX1262 drives its own TX/RX antenna
// switch through DIO2 (LoRaService sets USE_DIO2_ANT_SWITCH), while RXEN gates the
// RX LNA path. Park it HIGH so the radio can receive; TX is unaffected. LoRaService
// leaves RADIO_RXEN at -1, so nothing else touches this pin.
static constexpr int8_t NEWSHEEN_LORA_RXEN = 14;

// 8x WS2812B ring on GPIO16 (through the U5 SN74LVC1T45 level shifter). A short
// warm-white boot pulse confirms the Bit Pirate is alive on this screenless board.
// LedService does `FastLED = CFastLED()` before it (re)registers the strip when the
// user enters `led` mode, so this boot-time controller is harmless afterwards.
static constexpr uint8_t NEWSHEEN_RING_PIN   = 16;
static constexpr uint8_t NEWSHEEN_RING_COUNT = 8;
static CRGB newsheenBootRing[NEWSHEEN_RING_COUNT];

void NewsheenBoard::initialize() {
    // Enable the LoRa RX path (DIO2 still owns the TX/RX antenna switch).
    pinMode(NEWSHEEN_LORA_RXEN, OUTPUT);
    digitalWrite(NEWSHEEN_LORA_RXEN, HIGH);

    // Warm-white "awake" pulse on the ring, then release it dark.
    FastLED.addLeds<WS2812, NEWSHEEN_RING_PIN, GRB>(newsheenBootRing, NEWSHEEN_RING_COUNT);
    FastLED.setBrightness(64);
    for (auto& px : newsheenBootRing) px = CRGB(255, 160, 60);  // warm white
    FastLED.show();
    delay(400);
    FastLED.clear(true);

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
