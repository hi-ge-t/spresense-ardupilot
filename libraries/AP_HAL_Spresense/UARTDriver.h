#pragma once

#include <AP_HAL/UARTDriver.h>

#include "AP_HAL_Spresense_Namespace.h"
#include "SafeBringup.h"

class Spresense::UARTDriver : public AP_HAL::UARTDriver {
public:
    explicit UARTDriver(const char *device_path);

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;
    uint32_t get_baud_rate() const override;

protected:
    void _begin(uint32_t baud, uint16_t rx_space, uint16_t tx_space) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    bool configure(uint32_t baud);

    const char *_device_path;
    FileDescriptor _device;
    uint32_t _baud;
};
