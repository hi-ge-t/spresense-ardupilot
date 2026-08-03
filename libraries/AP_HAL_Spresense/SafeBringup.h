#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

namespace Spresense {

static constexpr const char *CONSOLE_DEVICE = "/dev/ttyS0";
static constexpr const char *TELEMETRY_DEVICE = "/dev/ttyACM0";
static constexpr const char *HIL_DEVICE = "/dev/ttyS2";
static constexpr const char *GNSS_DEVICE = "/dev/gps2";
static constexpr const char *STORAGE_PATH = "/mnt/sd0/APM/ardupilot.stg";
static constexpr size_t STORAGE_SIZE = 16384U;

class MonotonicClock {
public:
    bool micros(uint64_t &time_us) const;
    bool delay_microseconds(uint32_t delay_us) const;
};

class FileDescriptor {
public:
    FileDescriptor();
    explicit FileDescriptor(int fd);
    ~FileDescriptor();

    FileDescriptor(const FileDescriptor &) = delete;
    FileDescriptor &operator=(const FileDescriptor &) = delete;

    bool open_device(const char *path, int flags);
    void attach_for_test(int fd);
    void close();
    bool is_open() const;
    int native_fd() const;
    ssize_t read_some(void *buffer, size_t size) const;
    bool write_all(const void *buffer, size_t size, uint8_t retry_limit = 4U) const;

private:
    int _fd;
};

class StorageFile {
public:
    StorageFile();
    ~StorageFile();

    StorageFile(const StorageFile &) = delete;
    StorageFile &operator=(const StorageFile &) = delete;

    bool init(const char *path = STORAGE_PATH, size_t size = STORAGE_SIZE);
    void close();
    bool healthy() const;
    bool read_block(void *destination, size_t source, size_t size);
    bool write_block(size_t destination, const void *source, size_t size);

private:
    bool range_valid(size_t offset, size_t size) const;
    bool transfer(bool write_operation, size_t offset, void *buffer, size_t size);

    int _fd;
    size_t _size;
};

class OutputGuard {
public:
    bool request_arm();
    bool request_write(uint8_t channel, uint16_t period_us);

    bool armed() const;
    bool physical_output_enabled() const;
    uint32_t arm_reject_count() const;
    uint32_t write_reject_count() const;
    uint32_t physical_write_count() const;

private:
    uint32_t _arm_reject_count = 0;
    uint32_t _write_reject_count = 0;
};

} // namespace Spresense
