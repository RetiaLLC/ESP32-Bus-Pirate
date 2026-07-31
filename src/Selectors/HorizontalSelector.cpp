#include "HorizontalSelector.h"

HorizontalSelector::HorizontalSelector(
    IDeviceView& display,
    IInput& input,
    IUtilityService& utilityService)
    : display(display), input(input), utilityService(utilityService) {}

int HorizontalSelector::select(
    const std::string& title, 
    const std::vector<std::string>& options, 
    const std::string& description1, 
    const std::string& description2) {

    int currentIndex = 0;
    int lastIndex = -1;

    display.topBar(title, false, false);

    while (true) {
        if (lastIndex != currentIndex) {
            display.horizontalSelection(options, currentIndex, description1, description2);
            lastIndex = currentIndex;
        }

        char key = input.handler();

        switch (key) {
            case KEY_ARROW_LEFT:
                currentIndex = (currentIndex > 0) ? currentIndex - 1 : options.size() - 1;
                break;
             case KEY_ARROW_RIGHT:
            #if !defined(DEVICE_TDISPLAYS3)                
               currentIndex = (currentIndex < options.size() - 1) ? currentIndex + 1 : 0;
                break;
            case KEY_OK:
            #endif

                return currentIndex;
            default:
                break;
        }
    }
}

int HorizontalSelector::selectHeadless() {
    std::vector<std::string> options = {
        TerminalTypeEnumMapper::toString(TerminalTypeEnum::WiFiClient),
        TerminalTypeEnumMapper::toString(TerminalTypeEnum::WiFiAp),
        TerminalTypeEnumMapper::toString(TerminalTypeEnum::SerialPort),
    };

    const int n = static_cast<int>(options.size());
    const char* line1 = "LEFT/RIGHT: change   A: select";
    const char* line2 = "No input = USB Serial";

    int currentIndex = 2;  // default / pre-selected: USB Serial
    display.topBar("ESP32 BIT PIRATE", false, false);
    display.horizontalSelection(options, currentIndex, line1, line2);

    // Let the reset/boot button settle so a held key doesn't self-select.
    utilityService.sleepMs(250);

    // Interactive picker: d-pad LEFT/RIGHT changes, A selects. A generous
    // timeout (reset on every keypress) falls back to USB Serial if untouched.
    // Whatever the user picks is honoured — selecting WiFi is NOT overridden.
    const uint32_t timeoutMs = 6000;
    uint32_t deadline = utilityService.nowMs() + timeoutMs;
    while (utilityService.nowMs() < deadline) {
        char c = input.readChar();
        if (c == KEY_ARROW_LEFT) {
            currentIndex = (currentIndex - 1 + n) % n;
            display.horizontalSelection(options, currentIndex, line1, line2);
            deadline = utilityService.nowMs() + timeoutMs;
        } else if (c == KEY_ARROW_RIGHT) {
            currentIndex = (currentIndex + 1) % n;
            display.horizontalSelection(options, currentIndex, line1, line2);
            deadline = utilityService.nowMs() + timeoutMs;
        } else if (c == KEY_OK) {
            display.horizontalSelection(options, currentIndex, "Terminal selected", "Starting...");
            utilityService.sleepMs(250);
            return currentIndex;
        }
        utilityService.sleepMs(10);
    }

    return 2;  // timeout -> USB Serial
}
