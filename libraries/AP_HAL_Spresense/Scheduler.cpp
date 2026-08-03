#include "Scheduler.h"

#if defined(__NuttX__)

#include <AP_HAL/AP_HAL.h>

#include <algorithm>
#include <new>
#include <sched.h>
#include <string.h>
#include <unistd.h>
#ifdef __NuttX__
#include <sys/boardctl.h>
#endif

namespace {

constexpr int MAIN_PRIORITY = 180;
constexpr int TIMER_PRIORITY = 190;
constexpr int IO_PRIORITY = 160;
constexpr int UART_PRIORITY = 170;
constexpr int SENSOR_PRIORITY = 185;
constexpr uint32_t TIMER_PERIOD_US = 1000U;
constexpr uint32_t IO_PERIOD_US = 10000U;
constexpr size_t TIMER_STACK_BYTES = 8192U;
constexpr size_t IO_STACK_BYTES = 8192U;
constexpr size_t THREAD_STACK_MARGIN_BYTES = 2048U;
bool main_sample_wait_begin_reported;
bool main_sample_wait_return_reported;

void scheduler_marker(const char *marker)
{
    (void)write(STDOUT_FILENO, marker, strlen(marker));
}

int clamp_priority(int priority)
{
    return std::max(SCHED_PRIORITY_MIN,
                    std::min(SCHED_PRIORITY_MAX, priority));
}

bool start_thread(pthread_t &thread, void *(*entry)(void *), void *context,
                  const char *name, size_t stack_size, int priority)
{
    pthread_attr_t attributes {};
    if (pthread_attr_init(&attributes) != 0) {
        return false;
    }

    struct sched_param scheduling {};
    scheduling.sched_priority = clamp_priority(priority);
    const size_t actual_stack = std::max(static_cast<size_t>(PTHREAD_STACK_MIN),
                                         stack_size);
    const bool configured =
        pthread_attr_setstacksize(&attributes, actual_stack) == 0 &&
        pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED) == 0 &&
        pthread_attr_setschedpolicy(&attributes, SCHED_FIFO) == 0 &&
        pthread_attr_setschedparam(&attributes, &scheduling) == 0 &&
        pthread_attr_setinheritsched(&attributes, PTHREAD_EXPLICIT_SCHED) == 0;
    const int result = configured
        ? pthread_create(&thread, &attributes, entry, context)
        : -1;
    (void)pthread_attr_destroy(&attributes);
    if (result != 0) {
        return false;
    }
    if (name != nullptr) {
        (void)pthread_setname_np(thread, name);
    }
    return true;
}

int base_priority(AP_HAL::Scheduler::priority_base base)
{
    switch (base) {
    case AP_HAL::Scheduler::PRIORITY_BOOST:
    case AP_HAL::Scheduler::PRIORITY_MAIN:
        return MAIN_PRIORITY;
    case AP_HAL::Scheduler::PRIORITY_SPI:
    case AP_HAL::Scheduler::PRIORITY_I2C:
        return SENSOR_PRIORITY;
    case AP_HAL::Scheduler::PRIORITY_TIMER:
    case AP_HAL::Scheduler::PRIORITY_CAN:
        return TIMER_PRIORITY;
    case AP_HAL::Scheduler::PRIORITY_UART:
    case AP_HAL::Scheduler::PRIORITY_RCIN:
        return UART_PRIORITY;
    case AP_HAL::Scheduler::PRIORITY_RCOUT:
        // RCOutput is a reject-only implementation in this profile.
        return IO_PRIORITY;
    case AP_HAL::Scheduler::PRIORITY_LED:
    case AP_HAL::Scheduler::PRIORITY_IO:
    case AP_HAL::Scheduler::PRIORITY_STORAGE:
    case AP_HAL::Scheduler::PRIORITY_SCRIPTING:
    case AP_HAL::Scheduler::PRIORITY_NET:
        return IO_PRIORITY;
    }
    return IO_PRIORITY;
}

bool semaphore_ready(Spresense::Semaphore &semaphore)
{
    return semaphore.take_nonblocking() && semaphore.give();
}

} // namespace

void Spresense::Scheduler::init()
{
    uint64_t unused_time = 0U;
    _main_thread = pthread_self();
    _main_thread_valid = true;
    if (!semaphore_ready(_timer_semaphore) ||
        !semaphore_ready(_io_semaphore) ||
        !_clock.micros(unused_time) ||
        !start_thread(_timer_thread, timer_thread_trampoline, this,
                      "ap-timer", TIMER_STACK_BYTES, TIMER_PRIORITY) ||
        !start_thread(_io_thread, io_thread_trampoline, this,
                      "ap-io", IO_STACK_BYTES, IO_PRIORITY)) {
        mark_unhealthy();
    }
}

void Spresense::Scheduler::delay(uint16_t delay_ms)
{
    if (delay_ms == 0U) {
        return;
    }
    uint64_t now = 0U;
    if (!_clock.micros(now)) {
        mark_unhealthy();
        return;
    }
    const uint64_t deadline = now + static_cast<uint64_t>(delay_ms) * 1000ULL;
    while (now < deadline) {
        const uint32_t remaining = static_cast<uint32_t>(
            std::min<uint64_t>(deadline - now, 1000ULL));
        if (!_clock.delay_microseconds(remaining) || !_clock.micros(now)) {
            mark_unhealthy();
            return;
        }
        if (in_main_thread() && _min_delay_cb_ms <= delay_ms) {
            call_delay_cb();
        }
    }
}

void Spresense::Scheduler::delay_microseconds(uint16_t delay_us)
{
    const bool trace_sample_wait = initialized() && in_main_thread() &&
        delay_us == 100U && !main_sample_wait_return_reported;
    if (trace_sample_wait && !main_sample_wait_begin_reported) {
        main_sample_wait_begin_reported = true;
        scheduler_marker("SPRESENSE_M1_SCHEDULER=SAMPLE_WAIT_BEGIN\n");
    }
    if (!_clock.delay_microseconds(delay_us)) {
        mark_unhealthy();
    }
    if (trace_sample_wait && !main_sample_wait_return_reported) {
        main_sample_wait_return_reported = true;
        scheduler_marker("SPRESENSE_M1_SCHEDULER=SAMPLE_WAIT_RETURN\n");
    }
}

void Spresense::Scheduler::register_timer_process(AP_HAL::MemberProc process)
{
    if (!_timer_semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        mark_unhealthy();
        return;
    }
    for (uint8_t index = 0U; index < _timer_process_count; index++) {
        if (_timer_processes[index] == process) {
            if (!_timer_semaphore.give()) {
                mark_unhealthy();
            }
            return;
        }
    }
    if (_timer_process_count < MAX_TIMER_PROCESSES) {
        _timer_processes[_timer_process_count++] = process;
    } else {
        mark_unhealthy();
    }
    if (!_timer_semaphore.give()) {
        mark_unhealthy();
    }
}

void Spresense::Scheduler::register_io_process(AP_HAL::MemberProc process)
{
    if (!_io_semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        mark_unhealthy();
        return;
    }
    for (uint8_t index = 0U; index < _io_process_count; index++) {
        if (_io_processes[index] == process) {
            if (!_io_semaphore.give()) {
                mark_unhealthy();
            }
            return;
        }
    }
    if (_io_process_count < MAX_IO_PROCESSES) {
        _io_processes[_io_process_count++] = process;
    } else {
        mark_unhealthy();
    }
    if (!_io_semaphore.give()) {
        mark_unhealthy();
    }
}

void Spresense::Scheduler::register_timer_failsafe(AP_HAL::Proc process, uint32_t period_us)
{
    if (!_timer_semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        mark_unhealthy();
        return;
    }
    _failsafe = process;
    _failsafe_period_us = period_us;
    _last_failsafe_us = 0U;
    if (!_timer_semaphore.give()) {
        mark_unhealthy();
    }
}

void Spresense::Scheduler::set_system_initialized()
{
    if (initialized()) {
        AP_HAL::panic("Spresense scheduler initialized twice");
    }
    __atomic_store_n(&_system_initialized, true, __ATOMIC_RELEASE);
}

bool Spresense::Scheduler::is_system_initialized()
{
    return initialized();
}

void Spresense::Scheduler::reboot(bool hold_in_bootloader)
{
    AP_HAL::get_HAL().rcout->force_safety_on();
#ifdef __NuttX__
    (void)hold_in_bootloader;
    (void)boardctl(BOARDIOC_RESET, 0U);
#else
    (void)hold_in_bootloader;
#endif
    for (;;) {
        delay(1000U);
    }
}

bool Spresense::Scheduler::in_main_thread() const
{
    return _main_thread_valid && pthread_equal(pthread_self(), _main_thread);
}

bool Spresense::Scheduler::thread_create(
    AP_HAL::MemberProc process, const char *name, uint32_t stack_size,
    priority_base base, int8_t priority)
{
    auto *context = new (std::nothrow) ThreadContext {process};
    if (context == nullptr) {
        return false;
    }

    pthread_t thread {};
    const size_t actual_stack = static_cast<size_t>(stack_size) +
        THREAD_STACK_MARGIN_BYTES;
    if (!start_thread(thread, user_thread_trampoline, context, name,
                      actual_stack, base_priority(base) + priority)) {
        delete context;
        return false;
    }
    return true;
}

bool Spresense::Scheduler::timing_healthy() const
{
    return __atomic_load_n(&_timing_healthy, __ATOMIC_ACQUIRE);
}

void *Spresense::Scheduler::timer_thread_trampoline(void *context)
{
    static_cast<Scheduler *>(context)->timer_thread();
    return nullptr;
}

void *Spresense::Scheduler::io_thread_trampoline(void *context)
{
    static_cast<Scheduler *>(context)->io_thread();
    return nullptr;
}

void *Spresense::Scheduler::user_thread_trampoline(void *context)
{
    auto *thread_context = static_cast<ThreadContext *>(context);
    const AP_HAL::MemberProc process = thread_context->process;
    delete thread_context;
    process();
    return nullptr;
}

void Spresense::Scheduler::timer_thread()
{
    while (!initialized()) {
        if (!_clock.delay_microseconds(TIMER_PERIOD_US)) {
            mark_unhealthy();
        }
    }

    uint8_t uart_divider = 0U;
    for (;;) {
        if (!_clock.delay_microseconds(TIMER_PERIOD_US)) {
            mark_unhealthy();
            continue;
        }
        run_timer_processes();
        if (++uart_divider >= 10U) {
            uart_divider = 0U;
            const AP_HAL::HAL &hal = AP_HAL::get_HAL();
            for (uint8_t index = 0U; index < hal.num_serial; index++) {
                hal.serial(index)->_timer_tick();
            }
        }
    }
}

void Spresense::Scheduler::io_thread()
{
    while (!initialized()) {
        if (!_clock.delay_microseconds(IO_PERIOD_US)) {
            mark_unhealthy();
        }
    }

    for (;;) {
        if (!_clock.delay_microseconds(IO_PERIOD_US)) {
            mark_unhealthy();
            continue;
        }
        AP_HAL::get_HAL().storage->_timer_tick();
        run_io_processes();
    }
}

void Spresense::Scheduler::run_timer_processes()
{
    AP_HAL::MemberProc processes[MAX_TIMER_PROCESSES] {};
    AP_HAL::Proc failsafe = nullptr;
    uint32_t failsafe_period_us = 0U;
    uint8_t count = 0U;

    if (!_timer_semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        mark_unhealthy();
        return;
    }
    count = _timer_process_count;
    for (uint8_t index = 0U; index < count; index++) {
        processes[index] = _timer_processes[index];
    }
    failsafe = _failsafe;
    failsafe_period_us = _failsafe_period_us;
    if (!_timer_semaphore.give()) {
        mark_unhealthy();
    }

    for (uint8_t index = 0U; index < count; index++) {
        if (processes[index]) {
            processes[index]();
        }
    }

    if (failsafe != nullptr) {
        uint64_t now = 0U;
        if (!_clock.micros(now)) {
            mark_unhealthy();
        } else if (_last_failsafe_us == 0U || failsafe_period_us == 0U ||
                   now - _last_failsafe_us >= failsafe_period_us) {
            _last_failsafe_us = now;
            failsafe();
        }
    }
}

void Spresense::Scheduler::run_io_processes()
{
    AP_HAL::MemberProc processes[MAX_IO_PROCESSES] {};
    uint8_t count = 0U;

    if (!_io_semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        mark_unhealthy();
        return;
    }
    count = _io_process_count;
    for (uint8_t index = 0U; index < count; index++) {
        processes[index] = _io_processes[index];
    }
    if (!_io_semaphore.give()) {
        mark_unhealthy();
    }

    for (uint8_t index = 0U; index < count; index++) {
        if (processes[index]) {
            processes[index]();
        }
    }
}

void Spresense::Scheduler::mark_unhealthy()
{
    __atomic_store_n(&_timing_healthy, false, __ATOMIC_RELEASE);
}

bool Spresense::Scheduler::initialized() const
{
    return __atomic_load_n(&_system_initialized, __ATOMIC_ACQUIRE);
}

#else

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

void Spresense::Scheduler::register_timer_failsafe(
    AP_HAL::Proc process, uint32_t period_us)
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

bool Spresense::Scheduler::thread_create(
    AP_HAL::MemberProc process, const char *name, uint32_t stack_size,
    priority_base base, int8_t priority)
{
    (void)process;
    (void)name;
    (void)stack_size;
    (void)base;
    (void)priority;
    return false;
}

bool Spresense::Scheduler::timing_healthy() const
{
    return _timing_healthy;
}

#endif
