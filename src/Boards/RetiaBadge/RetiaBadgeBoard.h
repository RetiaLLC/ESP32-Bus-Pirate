#pragma once

#ifdef DEVICE_RETIA_BADGE

#include "Boards/Common/Views/Ili9341SpiDeviceView.h"
#include "Boards/RetiaBadge/RetiaBadgeInput.h"
#include "Boards/Common/Serial/BoardHostSerial.h"

// Retia 2024 DEF CON badge (ESP32-S3-WROOM-1, 8MB flash, 2MB quad PSRAM).
// ILI9341 240x320 TFT on the shared SPI bus (SCK13/MOSI11/MISO12), d-pad + A/B
// for on-screen navigation, native USB-CDC as the terminal link.
class RetiaBadgeBoard final {
public:
    RetiaBadgeBoard();

    void initialize();
    IDeviceView& getDeviceView();
    IInput& getDeviceInput();
    IHostSerial& getHostSerial();

private:
    static Ili9341SpiConfig createDisplayConfig();

    BoardHostSerial hostSerial;
    Ili9341SpiConfig displayConfig;
    Ili9341SpiDeviceView deviceView;
    RetiaBadgeInput deviceInput;
};

#endif
