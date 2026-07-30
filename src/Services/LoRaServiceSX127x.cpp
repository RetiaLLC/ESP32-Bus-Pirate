#ifdef DEVICE_RETIA_BADGE

#include "Services/LoRaServiceSX127x.h"
#include <Arduino.h>
#include <cmath>
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

// --- SX1276 register map (LoRa mode) ---
#define R_FIFO              0x00
#define R_OP_MODE           0x01
#define R_FRF_MSB           0x06
#define R_FRF_MID           0x07
#define R_FRF_LSB           0x08
#define R_PA_CONFIG         0x09
#define R_OCP               0x0b
#define R_LNA               0x0c
#define R_FIFO_ADDR_PTR     0x0d
#define R_FIFO_TX_BASE      0x0e
#define R_FIFO_RX_BASE      0x0f
#define R_FIFO_RX_CURRENT   0x10
#define R_IRQ_FLAGS         0x12
#define R_RX_NB_BYTES       0x13
#define R_PKT_SNR           0x19
#define R_PKT_RSSI          0x1a
#define R_RSSI              0x1b
#define R_MODEM_CONFIG_1    0x1d
#define R_MODEM_CONFIG_2    0x1e
#define R_PREAMBLE_MSB      0x20
#define R_PREAMBLE_LSB      0x21
#define R_PAYLOAD_LENGTH    0x22
#define R_MODEM_CONFIG_3    0x26
#define R_SYNC_WORD         0x39
#define R_DIO_MAPPING_1     0x40
#define R_VERSION           0x42
#define R_PA_DAC            0x4d

#define MODE_LORA           0x80
#define MODE_SLEEP          0x00
#define MODE_STDBY          0x01
#define MODE_TX             0x03
#define MODE_RXCONT         0x05
#define MODE_CAD            0x07

#define IRQ_TX_DONE         0x08
#define IRQ_RX_DONE         0x40
#define IRQ_CRC_ERR         0x20
#define IRQ_CAD_DONE        0x04
#define IRQ_CAD_DETECTED    0x01

#define RSSI_OFFSET         157   // HF band (>525 MHz), badge is 915 MHz US

// Other chip-selects on the badge's shared SPI bus (park HIGH while LoRa talks).
static const uint8_t PARK_CS[] = {47 /*TFT*/, 10 /*SD*/, 14 /*touch*/, 39, 37};

void LoRaServiceSX127x::parkOtherCs() {
    for (uint8_t p : PARK_CS) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
}

// Take the shared SPI pins as GPIO for bit-banging (refcounted).
void LoRaServiceSX127x::beginBus() {
    if (busDepth_++ > 0) return;
    parkOtherCs();
    pinMode(sck_, OUTPUT);  digitalWrite(sck_, LOW);
    pinMode(mosi_, OUTPUT); digitalWrite(mosi_, LOW);
    pinMode(miso_, INPUT);
    pinMode(cs_, OUTPUT);   digitalWrite(cs_, HIGH);
}

// Hand SCK/MOSI/MISO back to the display's FSPI (SPI2_HOST) peripheral so the
// ILI9341 keeps working after a radio transaction (refcounted).
void LoRaServiceSX127x::endBus() {
    if (--busDepth_ > 0) return;
    if (busDepth_ < 0) busDepth_ = 0;
    digitalWrite(cs_, HIGH);
    esp_rom_gpio_connect_out_signal(sck_,  FSPICLK_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(mosi_, FSPID_OUT_IDX,  false, false);
    pinMode(miso_, INPUT);
    esp_rom_gpio_connect_in_signal(miso_, FSPIQ_IN_IDX, false);
}

uint8_t LoRaServiceSX127x::xfer(uint8_t out) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; --i) {
        digitalWrite(mosi_, (out >> i) & 1);
        digitalWrite(sck_, HIGH);
        in = (uint8_t)((in << 1) | (digitalRead(miso_) & 1));
        digitalWrite(sck_, LOW);
    }
    return in;
}

void LoRaServiceSX127x::writeReg(uint8_t addr, uint8_t val) {
    beginBus();
    digitalWrite(cs_, LOW);
    xfer(addr | 0x80);
    xfer(val);
    digitalWrite(cs_, HIGH);
    endBus();
}

uint8_t LoRaServiceSX127x::readReg(uint8_t addr) {
    beginBus();
    digitalWrite(cs_, LOW);
    xfer(addr & 0x7f);
    uint8_t v = xfer(0x00);
    digitalWrite(cs_, HIGH);
    endBus();
    return v;
}

void LoRaServiceSX127x::setMode(uint8_t mode) {
    writeReg(R_OP_MODE, MODE_LORA | mode);
}

void LoRaServiceSX127x::explicitHeaderMode() {
    writeReg(R_MODEM_CONFIG_1, readReg(R_MODEM_CONFIG_1) & 0xfe);
}

bool LoRaServiceSX127x::configure(SPIClass& spi, uint8_t sck, uint8_t miso, uint8_t mosi,
                                  uint8_t cs, uint8_t rst, uint8_t busy, uint8_t dio1,
                                  const LoRaRadioProfile& profile) {
    (void)spi; (void)busy;
    sck_ = sck; miso_ = miso; mosi_ = mosi;
    cs_ = cs; rst_ = rst; dio0_ = dio1;   // badge maps DIO0 onto the "dio1" slot
    profile_ = profile;

    beginBus();

    // Hardware reset pulse
    pinMode(rst_, OUTPUT);
    digitalWrite(rst_, LOW);  delay(10);
    digitalWrite(rst_, HIGH); delay(10);

    uint8_t version = readReg(R_VERSION);
    if (version != 0x12) {
        lastError_ = -1;
        initialized_ = false;
        endBus();
        return false;
    }

    setMode(MODE_SLEEP);
    delay(5);
    writeReg(R_FIFO_TX_BASE, 0x00);
    writeReg(R_FIFO_RX_BASE, 0x00);
    writeReg(R_LNA, readReg(R_LNA) | 0x03);   // LNA boost on
    writeReg(R_OCP, 0x2B);                     // OCP ~ 100 mA

    applyProfile(profile_);
    setMode(MODE_STDBY);

    initialized_ = true;
    lastError_ = 0;
    endBus();
    return true;
}

void LoRaServiceSX127x::applyProfile(const LoRaRadioProfile& p) {
    beginBus();
    profile_ = p;
    setMode(MODE_STDBY);
    setFrequency(p.frequency);

    uint8_t bwIdx;
    if      (p.bandwidth >= 500) bwIdx = 9;
    else if (p.bandwidth >= 250) bwIdx = 8;
    else if (p.bandwidth >= 125) bwIdx = 7;
    else if (p.bandwidth >= 62)  bwIdx = 6;
    else if (p.bandwidth >= 41)  bwIdx = 5;
    else if (p.bandwidth >= 31)  bwIdx = 4;
    else if (p.bandwidth >= 20)  bwIdx = 3;
    else if (p.bandwidth >= 15)  bwIdx = 2;
    else if (p.bandwidth >= 10)  bwIdx = 1;
    else                         bwIdx = 0;

    uint8_t cr = p.codingRate; if (cr < 5) cr = 5; if (cr > 8) cr = 8;
    uint8_t crIdx = cr - 4;
    writeReg(R_MODEM_CONFIG_1, (uint8_t)((bwIdx << 4) | (crIdx << 1) | 0x00));

    uint8_t sf = p.spreadingFactor; if (sf < 6) sf = 6; if (sf > 12) sf = 12;
    writeReg(R_MODEM_CONFIG_2, (uint8_t)((sf << 4) | (p.crc ? 0x04 : 0x00)));

    float symbolMs = ((float)(1UL << sf) / (float)(p.bandwidth * 1000)) * 1000.0f;
    uint8_t mc3 = 0x04;                            // AgcAutoOn
    if (symbolMs > 16.0f) mc3 |= 0x08;             // LowDataRateOptimize
    writeReg(R_MODEM_CONFIG_3, mc3);

    writeReg(R_PREAMBLE_MSB, (uint8_t)((p.preambleLength >> 8) & 0xFF));
    writeReg(R_PREAMBLE_LSB, (uint8_t)(p.preambleLength & 0xFF));

    uint8_t sw;
    if (p.syncWord <= 0xFF) sw = (uint8_t)(p.syncWord & 0xFF);
    else sw = (uint8_t)(((p.syncWord & 0xF000) >> 8) | ((p.syncWord & 0x00F0) >> 4));
    if (sw == 0x00) sw = 0x12;
    writeReg(R_SYNC_WORD, sw);

    int8_t pw = p.power; if (pw < 2) pw = 2; if (pw > 17) pw = 17;
    writeReg(R_PA_DAC, 0x84);
    writeReg(R_PA_CONFIG, (uint8_t)(0x80 | (pw - 2)));

    writeReg(R_DIO_MAPPING_1, 0x00);
    endBus();
}

bool LoRaServiceSX127x::setFrequency(float frequencyMHz) {
    currentFrequency_ = frequencyMHz;
    uint64_t frf = (uint64_t)((double)frequencyMHz * 1000000.0 / 61.03515625);
    beginBus();
    writeReg(R_FRF_MSB, (uint8_t)((frf >> 16) & 0xFF));
    writeReg(R_FRF_MID, (uint8_t)((frf >> 8) & 0xFF));
    writeReg(R_FRF_LSB, (uint8_t)(frf & 0xFF));
    endBus();
    return true;
}

bool LoRaServiceSX127x::setModemProfile(const LoRaRadioProfile& profile) {
    if (!initialized_) return false;
    applyProfile(profile);
    return true;
}

bool LoRaServiceSX127x::send(const uint8_t* data, size_t length) {
    if (!initialized_) { lastError_ = -1; return false; }
    if (length == 0 || length > 255) { lastError_ = 2; return false; }

    beginBus();
    receiving_ = false;
    setMode(MODE_STDBY);
    explicitHeaderMode();
    writeReg(R_FIFO_TX_BASE, 0x00);
    writeReg(R_FIFO_ADDR_PTR, 0x00);

    digitalWrite(cs_, LOW);
    xfer(R_FIFO | 0x80);
    for (size_t i = 0; i < length; ++i) xfer(data[i]);
    digitalWrite(cs_, HIGH);

    writeReg(R_PAYLOAD_LENGTH, (uint8_t)length);
    writeReg(R_IRQ_FLAGS, 0xFF);
    setMode(MODE_TX);

    uint32_t start = millis();
    bool ok = true;
    while ((readReg(R_IRQ_FLAGS) & IRQ_TX_DONE) == 0) {
        if (millis() - start > 4000) { ok = false; break; }
        delay(1);
    }
    writeReg(R_IRQ_FLAGS, 0xFF);
    setMode(MODE_STDBY);
    endBus();

    if (!ok) { txErrors_++; lastError_ = 2; return false; }
    txPackets_++;
    lastError_ = 0;
    return true;
}

bool LoRaServiceSX127x::startReceive(bool boosted) {
    (void)boosted;
    if (!initialized_) { lastError_ = -1; return false; }
    beginBus();
    explicitHeaderMode();
    writeReg(R_FIFO_RX_BASE, 0x00);
    writeReg(R_FIFO_ADDR_PTR, 0x00);
    writeReg(R_IRQ_FLAGS, 0xFF);
    writeReg(R_DIO_MAPPING_1, 0x00);
    setMode(MODE_RXCONT);
    receiving_ = true;
    endBus();
    return true;
}

int16_t LoRaServiceSX127x::pollReceive(std::vector<uint8_t>& payload) {
    if (!initialized_) return RECEIVE_NOT_INITIALIZED;

    beginBus();
    uint8_t irq = readReg(R_IRQ_FLAGS);
    if ((irq & IRQ_RX_DONE) == 0) { endBus(); return RECEIVE_TIMEOUT; }

    writeReg(R_IRQ_FLAGS, irq);
    if (irq & IRQ_CRC_ERR) { endBus(); rxErrors_++; lastError_ = 2; return RECEIVE_ERROR; }

    uint8_t len = readReg(R_RX_NB_BYTES);
    uint8_t cur = readReg(R_FIFO_RX_CURRENT);
    writeReg(R_FIFO_ADDR_PTR, cur);

    payload.resize(len);
    digitalWrite(cs_, LOW);
    xfer(R_FIFO & 0x7f);
    for (uint8_t i = 0; i < len; ++i) payload[i] = xfer(0x00);
    digitalWrite(cs_, HIGH);

    int8_t snrRaw = (int8_t)readReg(R_PKT_SNR);
    lastSnr_ = snrRaw * 0.25f;
    int pktRssi = (int)readReg(R_PKT_RSSI) - RSSI_OFFSET;
    if (lastSnr_ < 0) pktRssi += (int)lastSnr_;
    lastRssi_ = (float)pktRssi;
    lastPacketLength_ = len;
    rxPackets_++;
    lastError_ = 0;
    endBus();
    return RECEIVE_OK;
}

void LoRaServiceSX127x::stopReceive() {
    receiving_ = false;
    if (initialized_) { beginBus(); setMode(MODE_STDBY); endBus(); }
}

int16_t LoRaServiceSX127x::receive(std::vector<uint8_t>& payload, uint32_t timeoutMs,
                                   bool boosted, bool countTimeout) {
    if (!initialized_) return RECEIVE_NOT_INITIALIZED;
    if (!receiving_) startReceive(boosted);

    uint32_t start = millis();
    for (;;) {
        int16_t r = pollReceive(payload);
        if (r == RECEIVE_OK || r == RECEIVE_ERROR) return r;
        if (timeoutMs != 0 && (millis() - start) >= timeoutMs) break;
        delay(2);
    }
    if (countTimeout) rxTimeouts_++;
    return RECEIVE_TIMEOUT;
}

bool LoRaServiceSX127x::measureRssi(float frequency, uint32_t durationMs, RssiStats& stats) {
    if (!initialized_) return false;
    float saved = currentFrequency_;
    beginBus();
    setFrequency(frequency);
    setMode(MODE_RXCONT);
    delay(5);

    int32_t sum = 0; uint32_t n = 0;
    int16_t mn = 32767, mx = -32768;
    uint32_t start = millis();
    while (millis() - start < durationMs) {
        int r = (int)readReg(R_RSSI) - RSSI_OFFSET;
        if (r < mn) mn = (int16_t)r;
        if (r > mx) mx = (int16_t)r;
        sum += r; n++;
        delay(2);
    }
    setMode(MODE_STDBY);
    setFrequency(saved);
    endBus();
    if (n == 0) return false;
    stats.minimum = mn;
    stats.maximum = mx;
    stats.average = (float)sum / (float)n;
    stats.samples = n;
    return true;
}

bool LoRaServiceSX127x::runCad(bool& detected, uint32_t timeoutMs) {
    detected = false;
    if (!initialized_) return false;
    beginBus();
    writeReg(R_IRQ_FLAGS, 0xFF);
    writeReg(R_DIO_MAPPING_1, 0x80);
    setMode(MODE_CAD);
    uint32_t start = millis();
    bool done = false;
    while (millis() - start < timeoutMs) {
        uint8_t irq = readReg(R_IRQ_FLAGS);
        if (irq & IRQ_CAD_DONE) {
            detected = (irq & IRQ_CAD_DETECTED) != 0;
            done = true;
            break;
        }
        delay(1);
    }
    writeReg(R_IRQ_FLAGS, 0xFF);
    writeReg(R_DIO_MAPPING_1, 0x00);
    setMode(MODE_STDBY);
    endBus();
    (void)done;
    return true;
}

bool LoRaServiceSX127x::startContinuousWave() {
    if (!initialized_) return false;
    beginBus();
    setMode(MODE_STDBY);
    writeReg(R_MODEM_CONFIG_2, readReg(R_MODEM_CONFIG_2) | 0x08);
    writeReg(R_PAYLOAD_LENGTH, 0xFF);
    setMode(MODE_TX);
    continuousWave_ = true;
    endBus();
    return true;
}

void LoRaServiceSX127x::stopContinuousWave() {
    if (!initialized_) return;
    beginBus();
    setMode(MODE_STDBY);
    writeReg(R_MODEM_CONFIG_2, readReg(R_MODEM_CONFIG_2) & ~0x08);
    continuousWave_ = false;
    endBus();
}

bool LoRaServiceSX127x::transmitFrame(const LoRaFrame& frame, bool& profileRestored) {
    profileRestored = false;
    if (!initialized_) { lastError_ = -1; return false; }
    LoRaRadioProfile saved = profile_;
    applyProfile(frame.profile);
    bool ok = send(frame.payload.data(), frame.payload.size());
    applyProfile(saved);
    profileRestored = true;
    return ok;
}

uint32_t LoRaServiceSX127x::getTimeOnAir(size_t payloadLength) const {
    float bw = (float)profile_.bandwidth * 1000.0f;
    uint8_t sf = profile_.spreadingFactor;
    float ts = (float)(1UL << sf) / bw;
    float tPreamble = ((float)profile_.preambleLength + 4.25f) * ts;
    int de = (ts * 1000.0f > 16.0f) ? 1 : 0;
    int cr = profile_.codingRate - 4;
    if (cr < 1) cr = 1; if (cr > 4) cr = 4;
    float num = 8.0f * (float)payloadLength - 4.0f * sf + 28.0f + 16.0f;
    float den = 4.0f * (float)(sf - 2 * de);
    float nb = ceilf(num / den) * (float)(cr + 4);
    if (nb < 0) nb = 0;
    float tPayload = (8.0f + nb) * ts;
    return (uint32_t)((tPreamble + tPayload) * 1000.0f);
}

void LoRaServiceSX127x::deinitRfModule() {
    if (initialized_) {
        beginBus();
        setMode(MODE_SLEEP);
        endBus();
    }
    initialized_ = false;
    receiving_ = false;
}

void LoRaServiceSX127x::resetStats() {
    txPackets_ = txErrors_ = 0;
    rxPackets_ = rxTimeouts_ = rxErrors_ = rxDropped_ = 0;
}

#endif // DEVICE_RETIA_BADGE
