#include "Semaphores.h"

#if defined(__NuttX__)

#include <errno.h>
#include <pthread.h>
#include <time.h>

namespace {

pthread_mutex_t *as_mutex(void *storage)
{
    return reinterpret_cast<pthread_mutex_t *>(storage);
}

pthread_cond_t *as_condition(void *storage)
{
    return reinterpret_cast<pthread_cond_t *>(storage);
}

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
    static_assert(sizeof(pthread_mutex_t) <= sizeof(_mutex_storage),
                  "Spresense mutex storage is too small");
    static_assert(alignof(pthread_mutex_t) <= alignof(Spresense::Semaphore),
                  "Spresense mutex storage alignment is too small");

    pthread_mutexattr_t attributes {};
    if (pthread_mutexattr_init(&attributes) != 0) {
        return;
    }
    if (pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT) == 0 &&
        pthread_mutexattr_settype(&attributes, PTHREAD_MUTEX_RECURSIVE) == 0 &&
        pthread_mutex_init(as_mutex(_mutex_storage), &attributes) == 0) {
        _initialized = true;
    }
    (void)pthread_mutexattr_destroy(&attributes);
}

Spresense::Semaphore::~Semaphore()
{
    if (_initialized) {
        (void)pthread_mutex_destroy(as_mutex(_mutex_storage));
    }
}

bool Spresense::Semaphore::give()
{
    return _initialized && pthread_mutex_unlock(as_mutex(_mutex_storage)) == 0;
}

bool Spresense::Semaphore::take(uint32_t timeout_ms)
{
    if (!_initialized) {
        return false;
    }
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return pthread_mutex_lock(as_mutex(_mutex_storage)) == 0;
    }

    struct timespec deadline {};
    if (clock_gettime(CLOCK_REALTIME, &deadline) != 0) {
        return false;
    }
    add_microseconds(deadline, static_cast<uint64_t>(timeout_ms) * 1000ULL);
    return pthread_mutex_timedlock(as_mutex(_mutex_storage), &deadline) == 0;
}

bool Spresense::Semaphore::take_nonblocking()
{
    return _initialized && pthread_mutex_trylock(as_mutex(_mutex_storage)) == 0;
}

Spresense::BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state),
    _pending(initial_state)
{
    static_assert(sizeof(pthread_mutex_t) <= sizeof(_mutex_storage),
                  "Spresense mutex storage is too small");
    static_assert(
        alignof(pthread_mutex_t) <= alignof(Spresense::BinarySemaphore),
        "Spresense mutex storage alignment is too small");
    static_assert(sizeof(pthread_cond_t) <= sizeof(_condition_storage),
                  "Spresense condition storage is too small");
    static_assert(
        alignof(pthread_cond_t) <= alignof(Spresense::BinarySemaphore),
        "Spresense condition storage alignment is too small");

    if (pthread_mutex_init(as_mutex(_mutex_storage), nullptr) == 0) {
        if (pthread_cond_init(as_condition(_condition_storage), nullptr) == 0) {
            _initialized = true;
        } else {
            (void)pthread_mutex_destroy(as_mutex(_mutex_storage));
        }
    }
}

Spresense::BinarySemaphore::~BinarySemaphore()
{
    if (_initialized) {
        (void)pthread_cond_destroy(as_condition(_condition_storage));
        (void)pthread_mutex_destroy(as_mutex(_mutex_storage));
    }
}

bool Spresense::BinarySemaphore::wait(uint32_t timeout_us)
{
    if (!_initialized || pthread_mutex_lock(as_mutex(_mutex_storage)) != 0) {
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
            result = pthread_cond_timedwait(as_condition(_condition_storage),
                                            as_mutex(_mutex_storage), &deadline);
        }
    }

    const bool acquired = result == 0 && _pending;
    if (acquired) {
        _pending = false;
    }
    (void)pthread_mutex_unlock(as_mutex(_mutex_storage));
    return acquired;
}

bool Spresense::BinarySemaphore::wait_blocking()
{
    if (!_initialized || pthread_mutex_lock(as_mutex(_mutex_storage)) != 0) {
        return false;
    }

    int result = 0;
    while (!_pending && result == 0) {
        result = pthread_cond_wait(as_condition(_condition_storage),
                                   as_mutex(_mutex_storage));
    }
    const bool acquired = result == 0 && _pending;
    if (acquired) {
        _pending = false;
    }
    (void)pthread_mutex_unlock(as_mutex(_mutex_storage));
    return acquired;
}

void Spresense::BinarySemaphore::signal()
{
    if (!_initialized || pthread_mutex_lock(as_mutex(_mutex_storage)) != 0) {
        return;
    }
    if (!_pending) {
        _pending = true;
        (void)pthread_cond_signal(as_condition(_condition_storage));
    }
    (void)pthread_mutex_unlock(as_mutex(_mutex_storage));
}

void Spresense::BinarySemaphore::signal_ISR()
{
    // No Spresense device backend calls this from an interrupt in the M1
    // output-disabled profile. Keep the same coalescing semantics as signal().
    signal();
}

#else

Spresense::Semaphore::Semaphore()
{
    // These members intentionally reserve the same ABI space as NuttX.
    (void)_mutex_storage;
    (void)_initialized;
}
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
    // These members intentionally reserve the same ABI space as NuttX.
    (void)_mutex_storage;
    (void)_condition_storage;
    (void)_initialized;
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
