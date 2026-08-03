#include "Storage.h"

#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#if defined(__NuttX__)
#include <unistd.h>

extern "C" bool board_sdcard_inserted(int slotno);
#endif

namespace {

constexpr uint8_t STORAGE_READY_ATTEMPTS = 31U;
constexpr uint32_t STORAGE_READY_DELAY_US = 100000U;

#if defined(__NuttX__)
void storage_marker(const char *marker)
{
    (void)write(STDOUT_FILENO, marker, strlen(marker));
}
#endif

} // namespace

Spresense::Storage::Storage(const char *path) :
    _path(path),
    _healthy(false)
{
}

void Spresense::Storage::init()
{
    if (strcmp(_path, STORAGE_PATH) != 0) {
        _healthy = _storage.init(_path, STORAGE_SIZE);
        return;
    }

    MonotonicClock clock;
    for (uint8_t attempt = 0U; attempt < STORAGE_READY_ATTEMPTS; attempt++) {
        const bool directory_ready =
            mkdir(STORAGE_DIRECTORY, 0777) == 0 || errno == EEXIST;
        if (directory_ready && _storage.init(_path, STORAGE_SIZE)) {
            _healthy = true;
            return;
        }
        if (attempt + 1U == STORAGE_READY_ATTEMPTS ||
            !clock.delay_microseconds(STORAGE_READY_DELAY_US)) {
            break;
        }
    }
    _healthy = false;
#if defined(__NuttX__)
    storage_marker(board_sdcard_inserted(0)
        ? "SPRESENSE_M1_STORAGE=CARD_PRESENT\n"
        : "SPRESENSE_M1_STORAGE=CARD_MISSING\n");
    storage_marker(access("/dev/mmcsd0", F_OK) == 0
        ? "SPRESENSE_M1_STORAGE=BLOCK_PRESENT\n"
        : "SPRESENSE_M1_STORAGE=BLOCK_MISSING\n");
    storage_marker(access("/mnt/sd0", F_OK) == 0
        ? "SPRESENSE_M1_STORAGE=MOUNT_PRESENT\n"
        : "SPRESENSE_M1_STORAGE=MOUNT_MISSING\n");
#endif
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
