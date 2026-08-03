#pragma once

#include <AP_HAL/Scheduler.h>

#include "AP_HAL_Spresense_Namespace.h"
#include "SafeBringup.h"

class Spresense::Scheduler : public AP_HAL::Scheduler {
public:
    void init() override;
    void delay(uint16_t delay_ms) override;
    void delay_microseconds(uint16_t delay_us) override;
    void register_timer_process(AP_HAL::MemberProc process) override;
    void register_io_process(AP_HAL::MemberProc process) override;
    void register_timer_failsafe(AP_HAL::Proc process, uint32_t period_us) override;
    void set_system_initialized() override;
    bool is_system_initialized() override;
    void reboot(bool hold_in_bootloader) override;
    bool in_main_thread() const override;

    bool timing_healthy() const;

private:
    MonotonicClock _clock;
    bool _system_initialized = false;
    bool _timing_healthy = true;
};
