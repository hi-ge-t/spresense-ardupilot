#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>

#include "AP_HAL_Spresense_Namespace.h"

class Spresense::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    ~Semaphore() override;

    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

private:
    // Keep the class layout identical in the Waf vehicle archive and the
    // Sony NuttX application.  The archive is compiled without __NuttX__, so
    // placing pthread_mutex_t here would create two incompatible ABIs.
    alignas(uint32_t) uint8_t _mutex_storage[28] {};
    bool _initialized = false;
    bool _taken = false;
};

class Spresense::BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    explicit BinarySemaphore(bool initial_state = false);
    ~BinarySemaphore() override;

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking() override;
    void signal() override;
    void signal_ISR() override;

private:
    // Sony NuttX 7fd61b2c: pthread_mutex_t=28 bytes,
    // pthread_cond_t=20 bytes, both aligned to 4 bytes.  Opaque storage keeps
    // the vehicle and application sides ABI-compatible.
    alignas(uint32_t) uint8_t _mutex_storage[28] {};
    alignas(uint32_t) uint8_t _condition_storage[20] {};
    bool _pending = false;
    bool _initialized = false;
};
