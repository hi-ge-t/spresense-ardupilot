#pragma once

#include <AP_HAL/Storage.h>

#include "AP_HAL_Spresense_Namespace.h"
#include "SafeBringup.h"

class Spresense::Storage : public AP_HAL::Storage {
public:
    explicit Storage(const char *path = STORAGE_PATH);

    void init() override;
    void read_block(void *destination, uint16_t source, size_t size) override;
    void write_block(uint16_t destination, const void *source, size_t size) override;
    bool healthy() override;

private:
    const char *_path;
    StorageFile _storage;
    bool _healthy;
};
