#pragma once

#if defined(DEVICE_RETIA_BADGE) || (defined(DEVICE_CUSTOM) && defined(CUSTOM_DISPLAY_DRIVER_ILI9341_SPI))

#include "Interfaces/IDeviceView.h"
#include "States/GlobalState.h"

#include <Arduino.h>
#include <LovyanGFX.hpp>

#define DARK_GREY_RECT 0x4208

// Hardware description for any SPI-connected ILI9341 panel.
// Mirrors St7789SpiConfig so the drawing code is identical; only the panel
// driver differs (ILI9341 vs ST7789). Defaults match the Retia DEF CON badge.
struct Ili9341SpiConfig {
  int8_t pinBacklight = -1;   // badge backlight is hardwired to 3V3 (not controllable)
  int8_t pinMiso = 12;
  int8_t pinMosi = 11;
  int8_t pinSclk = 13;
  int8_t pinCs = 47;
  int8_t pinDc = 40;
  int8_t pinReset = 41;
  int8_t pinPower = -1;
  spi_host_device_t spiHost = SPI2_HOST;

  uint16_t panelWidth = 240;
  uint16_t panelHeight = 320;
  uint16_t memoryWidth = 240;
  uint16_t memoryHeight = 320;
  uint16_t offsetX = 0;
  uint16_t offsetY = 0;

  uint32_t writeFrequency = 40000000;
  uint32_t readFrequency = 16000000;
  uint8_t rotation = 3;       // landscape 320x240, ribbon at top (badge is mounted 180 vs a T-Deck)
  bool invert = false;
  bool rgbOrder = false;      // ILI9341 is a BGR panel -> rgb_order = false
  bool powerActiveHigh = true;
  bool backlightActiveHigh = true;
  bool useSharedSpi = true;
  const char* selectionHelpLine1 = nullptr;
  const char* selectionHelpLine2 = nullptr;
};

// Lovyan driver
class LGFX_ILI9341SPI : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;

public:
  explicit LGFX_ILI9341SPI(const Ili9341SpiConfig& displayConfig) {
    {
    auto cfg = _bus.config();

    cfg.spi_host   = displayConfig.spiHost;
    cfg.spi_mode   = 0;

    cfg.freq_write = displayConfig.writeFrequency;
    cfg.freq_read  = displayConfig.readFrequency;

    cfg.pin_sclk = displayConfig.pinSclk;
    cfg.pin_mosi = displayConfig.pinMosi;
    cfg.pin_miso = displayConfig.pinMiso;
    cfg.pin_dc   = displayConfig.pinDc;

    cfg.spi_3wire = false;
    // No DMA: blocking flushes so the display never waits on a bus the LoRa radio
    // (bit-banged on the shared pins) may have momentarily taken over.
    cfg.dma_channel = 0;

    _bus.config(cfg);
    _panel.setBus(&_bus);
    }

    // --- PANEL
    {
    auto cfg = _panel.config();

    cfg.pin_cs   = displayConfig.pinCs;
    cfg.pin_rst  = displayConfig.pinReset;
    cfg.pin_busy = -1;

    cfg.panel_width   = displayConfig.panelWidth;
    cfg.panel_height  = displayConfig.panelHeight;
    cfg.memory_width  = displayConfig.memoryWidth;
    cfg.memory_height = displayConfig.memoryHeight;
    cfg.offset_x = displayConfig.offsetX;
    cfg.offset_y = displayConfig.offsetY;

    cfg.invert = displayConfig.invert;
    cfg.rgb_order = displayConfig.rgbOrder;

    cfg.dlen_16bit = false;
    _panel.config(cfg);
    }

    setPanel(&_panel);
  }

  // Re-initialise ONLY the SPI bus (not the panel) after the LoRa radio has
  // bit-banged the shared SCK/MOSI/MISO pins. Reclaims the bus for LovyanGFX
  // without a panel SWRESET, whose transient flashes the screen white — so a
  // continuous waterfall (bus handed back and forth every sweep) stays smooth.
  void reinitBus() { _bus.init(); }
};

class Ili9341SpiDeviceView : public IDeviceView {
public:
  explicit Ili9341SpiDeviceView(const Ili9341SpiConfig& config);

  void initialize() override;
  SPIClass& getSharedSpiInstance() override;
  void* getScreen() override;
  void logo() override;
  void welcome(TerminalTypeEnum& terminalType, std::string& terminalInfos) override;
  void show(PinoutConfig& config) override;
  void loading() override;
  void adapterMode(const std::string& adapterName, const std::string& description, const std::vector<std::string>& details) override;
  void clear() override;
  void drawLogicTrace(uint8_t pin, const std::vector<uint8_t>& buffer, uint8_t step) override;
  void drawAnalogicTrace(uint8_t pin, const std::vector<uint8_t>& buffer, uint8_t step) override;
  void drawWaterfall(const std::string& title, float startValue, float endValue, const char* unit, int rowIndex, int rowCount, int level) override;
  void setRotation(uint8_t rotation) override;
  void setBrightness(uint8_t brightness) override;
  uint8_t getBrightness() override;
  void topBar(const std::string& title, bool submenu, bool searchBar) override;
  void horizontalSelection(
    const std::vector<std::string>& options,
    uint16_t selectedIndex,
    const std::string& description1,
    const std::string& description2
  ) override;

  void renderDataScreen(const std::string& title, const std::vector<std::string>& lines) override;
  void renderSensorScreen(const std::string& title, const std::vector<std::string>& bigLines, const std::vector<std::string>& smallLines) override;
  void reacquireBus() override;

  void shutDown();

private:
  Ili9341SpiConfig config;
  LGFX_ILI9341SPI tft;
  uint8_t brightnessPct = 100;
  SPIClass sharedSpi{HSPI};
  std::string lastDataTitle;

  void drawCenterText(const std::string& text, int y, int fontSize);
  void welcomeWeb(const std::string& ip);
  void welcomeHotspot(const std::string& ip);
  void welcomeSerial(const std::string& baud);
};

#endif
