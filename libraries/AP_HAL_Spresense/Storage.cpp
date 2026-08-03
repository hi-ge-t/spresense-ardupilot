#include "Storage.h"

#include <errno.h>
#include <string.h>
#include <sys/stat.h>

Spresense::Storage::Storage(const char *path) :
    _path(path),
    _healthy(false)
{
}

void Spresense::Storage::init()
{
    if (strcmp(_path, STORAGE_PATH) == 0 &&
        mkdir(STORAGE_DIRECTORY, 0777) != 0 && errno != EEXIST) {
        _healthy = false;
        return;
    }
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
