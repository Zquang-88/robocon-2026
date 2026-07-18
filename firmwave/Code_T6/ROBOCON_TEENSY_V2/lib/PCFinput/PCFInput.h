#pragma once
#include <Arduino.h>
#include <Wire.h>
class PCFInput {
public:
    PCFInput(TwoWire &wire, uint8_t address);
    bool begin(uint32_t clock = 100000);
    bool update();
    // DIP
    bool dip1() const;
    bool dip2() const;
    bool dip3() const;
    bool dip4() const;
    uint8_t mode() const;
    // SW trạng thái đang nhấn
    bool sw1() const;
    bool sw2() const;
    bool sw3() const;
    bool sw4() const;
    // SW sự kiện nhấn 1 lần
    bool sw1Pressed() const;
    bool sw2Pressed() const;
    bool sw3Pressed() const;
    bool sw4Pressed() const;
    // SW sự kiện nhả 1 lần
    bool sw1Released() const;
    bool sw2Released() const;
    bool sw3Released() const;
    bool sw4Released() const;
    uint8_t raw() const;
    bool ok() const;

private:
    TwoWire *_wire;
    uint8_t _address;
    uint8_t _rawData;
    bool _ok;
    static const uint8_t NUM_SW = 4;
    bool _swStable[NUM_SW];
    bool _swLastStable[NUM_SW];
    bool _swReading[NUM_SW];
    bool _swPressed[NUM_SW];
    bool _swReleased[NUM_SW];
    uint32_t _lastDebounceTime[NUM_SW];
    uint16_t _debounceMs;
    bool readRaw();
    bool activeLow(uint8_t bit) const;
    bool getSW(uint8_t index) const;
    bool getSWPressed(uint8_t index) const;
    bool getSWReleased(uint8_t index) const;
    void updateButton(uint8_t index, bool readingNow);
};