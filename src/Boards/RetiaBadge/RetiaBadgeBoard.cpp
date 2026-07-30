#ifdef DEVICE_RETIA_BADGE

#include "Boards/RetiaBadge/RetiaBadgeBoard.h"
#include <Arduino.h>

// Other chip-selects that hang off the badge's single shared SPI bus. They must
// be parked HIGH before the display starts, otherwise a floating CS (the LoRa
// radio's especially) corrupts bus traffic. See docs/pinout.md "shared SPI bus".
static constexpr int8_t RETIA_CS_LORA      = 48;
static constexpr int8_t RETIA_CS_SD        = 10;
static constexpr int8_t RETIA_CS_TOUCH     = 14;
static constexpr int8_t RETIA_CS_MODULE_SD = 39;
static constexpr int8_t RETIA_CS_ACCESSORY = 37;

Ili9341SpiConfig RetiaBadgeBoard::createDisplayConfig() {
    Ili9341SpiConfig config;

    // Shared SPI bus (SPI2_HOST): SCK 13 / MOSI 11 / MISO 12.
    config.pinSclk = 13;
    config.pinMosi = 11;
    config.pinMiso = 12;

    // ILI9341 control lines.
    config.pinCs    = 47;
    config.pinDc    = 40;
    config.pinReset = 41;

    // Backlight is hardwired to 3V3 on the badge - not software controllable.
    config.pinBacklight = -1;
    config.pinPower = -1;

    // 240x320 panel driven in landscape. Rotation 3 puts the origin so the UI is
    // upright with the ribbon at the top on this board (mounted 180 vs a T-Deck).
    config.panelWidth   = 240;
    config.panelHeight  = 320;
    config.memoryWidth  = 240;
    config.memoryHeight = 320;
    config.offsetX = 0;
    config.offsetY = 0;
    config.rotation = 3;

    config.writeFrequency = 40000000;   // pinout notes the TFT runs fine at 40 MHz
    config.readFrequency  = 16000000;
    config.invert   = false;
    config.rgbOrder = false;            // ILI9341 is BGR
    config.useSharedSpi = true;

    config.selectionHelpLine1 = "LEFT / RIGHT to change";
    config.selectionHelpLine2 = "A to accept";

    return config;
}

RetiaBadgeBoard::RetiaBadgeBoard()
    : displayConfig(createDisplayConfig()),
      deviceView(displayConfig) {}

void RetiaBadgeBoard::initialize() {
    // Park every other CS on the shared bus HIGH before the display talks.
    const int8_t idleCs[] = {
        RETIA_CS_LORA, RETIA_CS_SD, RETIA_CS_TOUCH, RETIA_CS_MODULE_SD, RETIA_CS_ACCESSORY
    };
    for (int8_t cs : idleCs) {
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
    }

    deviceView.initialize();
    deviceView.logo();
    deviceInput.waitPress(3000);
    deviceView.clear();
}

IDeviceView& RetiaBadgeBoard::getDeviceView() {
    return deviceView;
}

IInput& RetiaBadgeBoard::getDeviceInput() {
    return deviceInput;
}

IHostSerial& RetiaBadgeBoard::getHostSerial() {
    return hostSerial;
}

#endif
