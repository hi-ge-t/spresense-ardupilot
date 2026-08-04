#pragma once

#include <AP_HAL_Empty/Util.h>

#include "AP_HAL_Spresense_Namespace.h"

class Spresense::Util : public Empty::Util {
public:
    // Hardware RTC integration is outside the output-disabled M1 link gate.
    void set_hw_rtc(uint64_t time_utc_usec) override
    {
        (void)time_utc_usec;
    }

    uint64_t get_hw_rtc() const override
    {
        return 0U;
    }
};
