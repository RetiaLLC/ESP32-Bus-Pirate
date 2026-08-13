#pragma once

#ifdef DEVICE_NEWSHEEN

#include "Boards/Common/Views/NoScreenDeviceView.h"
#include "Boards/Newsheen/NewsheenInput.h"
#include "Boards/Common/Serial/BoardHostSerial.h"

// Retia "Newsheen" - the ESP32-S3 Pusheen puck (esp32_base_puck_v2, N16R2:
// 16MB quad flash / 2MB quad PSRAM) that drives the Pusheen silicone cat lamp.
//
// The puck is SCREENLESS: its only on-board output is the ring of 8x WS2812B on
// GPIO16 (behind an SN74LVC1T45 level shifter), so the Bit Pirate UI lives on the
// USB-CDC / Wi-Fi terminal - there is no on-screen menu. Boot auto-selects USB
// serial (see TerminalTypeConfigurator). The ring is driven only via the `led`
// command (LedService owns FastLED on GPIO16); the board does NOT touch it at boot.
//
// On-board hardware: Wio-SX1262 LoRa (a REAL SX126x with BUSY+DIO1, so it runs the
// native LoRaService - unlike the badge's DIO-less RFM95W), an IR receiver on
// GPIO4, the SW3 user button (GPIO17) + BOOT (GPIO0), and an I2C/I2S sensor header
// (J3: SDA35/SCL36, I2S 37/38/39).
class NewsheenBoard final {
public:
    void initialize();
    IDeviceView& getDeviceView();
    IInput& getDeviceInput();
    IHostSerial& getHostSerial();

private:
    BoardHostSerial hostSerial;
    NoScreenDeviceView deviceView;
    NewsheenInput deviceInput;
};

#endif
