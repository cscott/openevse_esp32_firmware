/*
 * Author: C. Scott Ananian
 */

#if defined(ENABLE_CAP1214)

#ifndef CAP1214_H
#define CAP1214_H

#include "rfid.h"
#include <MicroTasks.h>

class CAP1214 : public RfidReader, public MicroTasks::Task {
private:
    std::function<void(String &uid)> onCardDetected = [] (String&) {};

    enum class DeviceStatus {
        BOOTUP,
        ACTIVE,
        NOT_ACTIVE,
        FAILED
    };

    DeviceStatus status = DeviceStatus::BOOTUP;

    void initialize();

    bool write(uint8_t regno, uint8_t val);
    bool read(uint8_t regno, uint8_t *val);
    uint16_t read_buttons();
    void read_key(char *key, bool *prox);

    ulong sleepCount = 0;
    bool is_awake = true;
    String pin = String('\0');

protected:
    void setup() { }
    unsigned long loop(MicroTasks::WakeReason reason);

public:
    CAP1214();
    void begin();

    void setOnCardDetected(std::function<void(String&)> onCardDet) override {onCardDetected = onCardDet;}
    bool readerFailure() override;
};

extern CAP1214 cap1214;

#endif
#endif
