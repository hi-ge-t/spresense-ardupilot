#pragma once

#include <AP_HAL/RCOutput.h>

#include "AP_HAL_Spresense_Namespace.h"
#include "SafeBringup.h"

class Spresense::RCOutput : public AP_HAL::RCOutput {
public:
    void init() override;
    void set_freq(uint32_t channel_mask, uint16_t frequency_hz) override;
    uint16_t get_freq(uint8_t channel) override;
    void enable_ch(uint8_t channel) override;
    void disable_ch(uint8_t channel) override;
    void write(uint8_t channel, uint16_t period_us) override;
    uint16_t read(uint8_t channel) override;
    void read(uint16_t *period_us, uint8_t length) override;
    void cork() override;
    void push() override;

    const OutputGuard &guard() const;

private:
    OutputGuard _guard;
};
