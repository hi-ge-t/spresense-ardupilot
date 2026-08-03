#pragma once

#include <AP_HAL/Scheduler.h>

#include "AP_HAL_Spresense_Namespace.h"
#include "SafeBringup.h"

#if defined(__NuttX__)
#include <pthread.h>
#include "Semaphores.h"
#endif

class Spresense::Scheduler : public AP_HAL::Scheduler {
public:
    void init() override;
    void hal_initialized();
    void delay(uint16_t delay_ms) override;
    void delay_microseconds(uint16_t delay_us) override;
    void register_timer_process(AP_HAL::MemberProc process) override;
    void register_io_process(AP_HAL::MemberProc process) override;
    void register_timer_failsafe(AP_HAL::Proc process, uint32_t period_us) override;
    void set_system_initialized() override;
    bool is_system_initialized() override;
    void reboot(bool hold_in_bootloader) override;
    bool in_main_thread() const override;
    bool thread_create(AP_HAL::MemberProc process, const char *name,
                       uint32_t stack_size, priority_base base,
                       int8_t priority) override;

    bool timing_healthy() const;

private:
#if defined(__NuttX__)
    static constexpr uint8_t MAX_TIMER_PROCESSES = 12U;
    static constexpr uint8_t MAX_IO_PROCESSES = 12U;

    struct ThreadContext {
        AP_HAL::MemberProc process;
    };

    static void *timer_thread_trampoline(void *context);
    static void *io_thread_trampoline(void *context);
    static void *user_thread_trampoline(void *context);

    void timer_thread();
    void io_thread();
    void run_timer_processes();
    void run_io_processes();
    void mark_unhealthy();
    bool hal_ready() const;
    bool initialized() const;

    MonotonicClock _clock;
    Semaphore _timer_semaphore;
    Semaphore _io_semaphore;
    AP_HAL::MemberProc _timer_processes[MAX_TIMER_PROCESSES] {};
    AP_HAL::MemberProc _io_processes[MAX_IO_PROCESSES] {};
    AP_HAL::Proc _failsafe = nullptr;
    uint32_t _failsafe_period_us = 0U;
    uint64_t _last_failsafe_us = 0U;
    pthread_t _main_thread {};
    pthread_t _timer_thread {};
    pthread_t _io_thread {};
    uint8_t _timer_process_count = 0U;
    uint8_t _io_process_count = 0U;
    bool _system_initialized = false;
    bool _hal_initialized = false;
    bool _main_thread_valid = false;
    bool _timing_healthy = true;
#else
    MonotonicClock _clock;
    bool _system_initialized = false;
    bool _hal_initialized = false;
    bool _timing_healthy = true;
#endif
};
