#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE

#include <stdarg.h>
#include <stdio.h>
#include <time.h>

namespace {

uint64_t monotonic_microseconds()
{
    struct timespec now {};
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return 0U;
    }
    return static_cast<uint64_t>(now.tv_sec) * 1000000ULL
        + static_cast<uint64_t>(now.tv_nsec) / 1000ULL;
}

uint64_t start_time_us;

} // namespace

void AP_HAL::init()
{
    start_time_us = monotonic_microseconds();
}

void AP_HAL::panic(const char *errormsg, ...)
{
    va_list arguments;
    va_start(arguments, errormsg);
    vfprintf(stderr, errormsg, arguments);
    va_end(arguments);
    fputc('\n', stderr);

    for (;;) {
        struct timespec delay {1, 0};
        (void)nanosleep(&delay, nullptr);
    }
}

uint64_t AP_HAL::micros64()
{
    return monotonic_microseconds() - start_time_us;
}

uint64_t AP_HAL::millis64()
{
    return micros64() / 1000ULL;
}

uint32_t AP_HAL::micros()
{
    return static_cast<uint32_t>(micros64());
}

uint32_t AP_HAL::millis()
{
    return static_cast<uint32_t>(millis64());
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE
