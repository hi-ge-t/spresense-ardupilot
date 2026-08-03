#include "Scheduler.h"

void Spresense::Scheduler::init()
{
    uint64_t unused_time = 0U;
    _timing_healthy = _clock.micros(unused_time);
}

void Spresense::Scheduler::delay(uint16_t delay_ms)
{
    if (!_clock.delay_microseconds(static_cast<uint32_t>(delay_ms) * 1000U)) {
        _timing_healthy = false;
    }
}

void Spresense::Scheduler::delay_microseconds(uint16_t delay_us)
{
    if (!_clock.delay_microseconds(delay_us)) {
        _timing_healthy = false;
    }
}

void Spresense::Scheduler::register_timer_process(AP_HAL::MemberProc process)
{
    (void)process;
    _timing_healthy = false;
}

void Spresense::Scheduler::register_io_process(AP_HAL::MemberProc process)
{
    (void)process;
    _timing_healthy = false;
}

void Spresense::Scheduler::register_timer_failsafe(AP_HAL::Proc process, uint32_t period_us)
{
    (void)process;
    (void)period_us;
    _timing_healthy = false;
}

void Spresense::Scheduler::set_system_initialized()
{
    _system_initialized = true;
}

bool Spresense::Scheduler::is_system_initialized()
{
    return _system_initialized;
}

void Spresense::Scheduler::reboot(bool hold_in_bootloader)
{
    (void)hold_in_bootloader;
    _system_initialized = false;
    for (;;) {
        delay(1000U);
    }
}

bool Spresense::Scheduler::in_main_thread() const
{
    return true;
}

bool Spresense::Scheduler::timing_healthy() const
{
    return _timing_healthy;
}
