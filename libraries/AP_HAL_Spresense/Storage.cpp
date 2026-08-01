#include "Storage.h"

#include <string.h>

Spresense::Storage::Storage(const char *path) :
    _path(path),
    _healthy(false)
{
}

void Spresense::Storage::init()
{
    _healthy = _storage.init(_path, STORAGE_SIZE);
}

void Spresense::Storage::read_block(void *destination, uint16_t source, size_t size)
{
    if (!_healthy || !_storage.read_block(destination, source, size)) {
        _healthy = false;
        if (destination != nullptr) {
            memset(destination, 0, size);
        }
    }
}

void Spresense::Storage::write_block(uint16_t destination, const void *source, size_t size)
{
    if (!_healthy || !_storage.write_block(destination, source, size)) {
        _healthy = false;
    }
}

bool Spresense::Storage::healthy()
{
    return _healthy && _storage.healthy();
}
