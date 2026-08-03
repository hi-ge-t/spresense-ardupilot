#include "Semaphores.h"

#if defined(__NuttX__)

#include <errno.h>
#include <time.h>

namespace {

void add_microseconds(struct timespec &value, uint64_t timeout_us)
{
    value.tv_sec += static_cast<time_t>(timeout_us / 1000000U);
    value.tv_nsec += static_cast<long>(timeout_us % 1000000U) * 1000L;
    if (value.tv_nsec >= 1000000000L) {
        value.tv_sec++;
        value.tv_nsec -= 1000000000L;
    }
}

} // namespace

Spresense::Semaphore::Semaphore()
{
    pthread_mutexattr_t attributes {};
    if (pthread_mutexattr_init(&attributes) != 0) {
        return;
    }
    if (pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT) == 0 &&
        pthread_mutexattr_settype(&attributes, PTHREAD_MUTEX_RECURSIVE) == 0 &&
        pthread_mutex_init(&_mutex, &attributes) == 0) {
        _initialized = true;
    }
    (void)pthread_mutexattr_destroy(&attributes);
}

Spresense::Semaphore::~Semaphore()
{
    if (_initialized) {
        (void)pthread_mutex_destroy(&_mutex);
    }
}

bool Spresense::Semaphore::give()
{
    return _initialized && pthread_mutex_unlock(&_mutex) == 0;
}

bool Spresense::Semaphore::take(uint32_t timeout_ms)
{
    if (!_initialized) {
        return false;
    }
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return pthread_mutex_lock(&_mutex) == 0;
    }

    struct timespec deadline {};
    if (clock_gettime(CLOCK_REALTIME, &deadline) != 0) {
        return false;
    }
    add_microseconds(deadline, static_cast<uint64_t>(timeout_ms) * 1000ULL);
    return pthread_mutex_timedlock(&_mutex, &deadline) == 0;
}

bool Spresense::Semaphore::take_nonblocking()
{
    return _initialized && pthread_mutex_trylock(&_mutex) == 0;
}

Spresense::BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state),
    _pending(initial_state)
{
    if (pthread_mutex_init(&_mutex, nullptr) == 0) {
        if (pthread_cond_init(&_condition, nullptr) == 0) {
            _initialized = true;
        } else {
            (void)pthread_mutex_destroy(&_mutex);
        }
    }
}

Spresense::BinarySemaphore::~BinarySemaphore()
{
    if (_initialized) {
        (void)pthread_cond_destroy(&_condition);
        (void)pthread_mutex_destroy(&_mutex);
    }
}

bool Spresense::BinarySemaphore::wait(uint32_t timeout_us)
{
    if (!_initialized || pthread_mutex_lock(&_mutex) != 0) {
        return false;
    }

    struct timespec deadline {};
    int result = 0;
    if (timeout_us != 0U) {
        if (clock_gettime(CLOCK_REALTIME, &deadline) != 0) {
            result = EINVAL;
        } else {
            add_microseconds(deadline, timeout_us);
        }
    }
    while (!_pending && result == 0) {
        if (timeout_us == 0U) {
            result = ETIMEDOUT;
        } else {
            result = pthread_cond_timedwait(
                &_condition, &_mutex, &deadline);
        }
    }

    const bool acquired = result == 0 && _pending;
    if (acquired) {
        _pending = false;
    }
    (void)pthread_mutex_unlock(&_mutex);
    return acquired;
}

bool Spresense::BinarySemaphore::wait_blocking()
{
    if (!_initialized || pthread_mutex_lock(&_mutex) != 0) {
        return false;
    }

    int result = 0;
    while (!_pending && result == 0) {
        result = pthread_cond_wait(&_condition, &_mutex);
    }
    const bool acquired = result == 0 && _pending;
    if (acquired) {
        _pending = false;
    }
    (void)pthread_mutex_unlock(&_mutex);
    return acquired;
}

void Spresense::BinarySemaphore::signal()
{
    if (!_initialized || pthread_mutex_lock(&_mutex) != 0) {
        return;
    }
    if (!_pending) {
        _pending = true;
        (void)pthread_cond_signal(&_condition);
    }
    (void)pthread_mutex_unlock(&_mutex);
}

void Spresense::BinarySemaphore::signal_ISR()
{
    // No Spresense device backend calls this from an interrupt in the M1
    // output-disabled profile. Keep the same coalescing semantics as signal().
    signal();
}

#else

Spresense::Semaphore::Semaphore() = default;
Spresense::Semaphore::~Semaphore() = default;

bool Spresense::Semaphore::give()
{
    if (!_taken) {
        return false;
    }
    _taken = false;
    return true;
}

bool Spresense::Semaphore::take(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return take_nonblocking();
}

bool Spresense::Semaphore::take_nonblocking()
{
    if (_taken) {
        return false;
    }
    _taken = true;
    return true;
}

Spresense::BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state),
    _pending(initial_state)
{
}

Spresense::BinarySemaphore::~BinarySemaphore() = default;

bool Spresense::BinarySemaphore::wait(uint32_t timeout_us)
{
    (void)timeout_us;
    if (!_pending) {
        return false;
    }
    _pending = false;
    return true;
}

bool Spresense::BinarySemaphore::wait_blocking()
{
    return wait(0U);
}

void Spresense::BinarySemaphore::signal()
{
    _pending = true;
}

void Spresense::BinarySemaphore::signal_ISR()
{
    signal();
}

#endif
