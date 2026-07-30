#pragma once

// SX127x (RFM95W / SX1276) LoRa service for the Retia DEF CON badge.
// The stock LoRaService targets SX126x (needs DIO1/BUSY); the badge radio is an
// SX1276 with only DIO0 routed, so this poll-based driver implements the same
// ILoRaService contract over the shared SPI bus.
#ifdef DEVICE_RETIA_BADGE

#include "Interfaces/ILoRaService.h"
#include <Arduino.h>
#include <SPI.h>
#include <vector>

class LoRaServiceSX127x : public ILoRaService {
public:
    bool configure(SPIClass& spi, uint8_t sck, uint8_t miso, uint8_t mosi,
                   uint8_t cs, uint8_t rst, uint8_t busy, uint8_t dio1,
                   const LoRaRadioProfile& profile) override;
    void deinitRfModule() override;

    bool send(const uint8_t* data, size_t length) override;
    bool startContinuousWave() override;
    void stopContinuousWave() override;

    bool startReceive(bool boosted = true) override;
    int16_t pollReceive(std::vector<uint8_t>& payload) override;
    void stopReceive() override;
    bool isReceiving() const override { return receiving_; }
    int16_t receive(std::vector<uint8_t>& payload, uint32_t timeoutMs,
                    bool boosted = true, bool countTimeout = true) override;

    bool setFrequency(float frequency) override;
    bool setModemProfile(const LoRaRadioProfile& profile) override;
    bool transmitFrame(const LoRaFrame& frame, bool& profileRestored) override;
    LoRaRadioProfile getProfile() const override { return profile_; }
    bool measureRssi(float frequency, uint32_t durationMs, RssiStats& stats) override;
    bool runCad(bool& detected, uint32_t timeoutMs = 250) override;
    uint32_t getTimeOnAir(size_t payloadLength) const override;

    bool isInitialized() const override { return initialized_; }
    float getCurrentFrequency() const override { return currentFrequency_; }
    float getRssi() const override { return lastRssi_; }
    float getSnr() const override { return lastSnr_; }
    size_t getLastPacketLength() const override { return lastPacketLength_; }
    int16_t getLastError() const override { return lastError_; }
    uint32_t getTxPackets() const override { return txPackets_; }
    uint32_t getTxErrors() const override { return txErrors_; }
    uint32_t getRxPackets() const override { return rxPackets_; }
    uint32_t getRxTimeouts() const override { return rxTimeouts_; }
    uint32_t getRxErrors() const override { return rxErrors_; }
    uint32_t getRxDropped() const override { return rxDropped_; }
    void resetStats() override;

private:
    void applyProfile(const LoRaRadioProfile& p);
    void writeReg(uint8_t addr, uint8_t val);
    uint8_t readReg(uint8_t addr);
    void setMode(uint8_t mode);
    void explicitHeaderMode();
    void parkOtherCs();

    // Bit-banged SPI to the radio + hand the shared bus back to the display's
    // FSPI routing afterwards (refcounted so nested calls restore only once).
    void beginBus();
    void endBus();
    uint8_t xfer(uint8_t out);
    int busDepth_ = 0;

    uint8_t sck_ = 13, miso_ = 12, mosi_ = 11, cs_ = 48, rst_ = 38, dio0_ = 21;
    LoRaRadioProfile profile_{};

    bool initialized_ = false;
    bool receiving_ = false;
    bool continuousWave_ = false;
    float currentFrequency_ = 915.0f;
    float lastRssi_ = 0.0f;
    float lastSnr_ = 0.0f;
    size_t lastPacketLength_ = 0;
    int16_t lastError_ = 0;

    uint32_t txPackets_ = 0, txErrors_ = 0;
    uint32_t rxPackets_ = 0, rxTimeouts_ = 0, rxErrors_ = 0, rxDropped_ = 0;

    SPISettings spiSettings_{8000000, MSBFIRST, SPI_MODE0};
};

#endif // DEVICE_RETIA_BADGE
