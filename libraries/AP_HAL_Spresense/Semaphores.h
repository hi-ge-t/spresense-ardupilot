#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>

#if defined(__NuttX__)
#include <pthread.h>
#endif

#include "AP_HAL_Spresense_Namespace.h"

class Spresense::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    ~Semaphore() override;

    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

private:
#if defined(__NuttX__)
    pthread_mutex_t _mutex {};
    bool _initialized = false;
#else
    bool _taken = false;
#endif
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
#if defined(__NuttX__)
    pthread_mutex_t _mutex {};
    pthread_cond_t _condition {};
    bool _pending = false;
    bool _initialized = false;
#else
    bool _pending = false;
#endif
};
