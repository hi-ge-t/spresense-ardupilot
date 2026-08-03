#include "SafeBringup.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

namespace Spresense {

#if defined(__NuttX__)
namespace {

void storage_init_marker(const char *marker)
{
    (void)::write(STDOUT_FILENO, marker, strlen(marker));
}

} // namespace
#endif

bool MonotonicClock::micros(uint64_t &time_us) const
{
    struct timespec now {};
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return false;
    }

    time_us = static_cast<uint64_t>(now.tv_sec) * 1000000ULL;
    time_us += static_cast<uint64_t>(now.tv_nsec) / 1000ULL;
    return true;
}

bool MonotonicClock::delay_microseconds(uint32_t delay_us) const
{
    struct timespec requested {};
    requested.tv_sec = static_cast<time_t>(delay_us / 1000000U);
    requested.tv_nsec = static_cast<long>(delay_us % 1000000U) * 1000L;

    while (nanosleep(&requested, &requested) != 0) {
        if (errno != EINTR) {
            return false;
        }
    }
    return true;
}

FileDescriptor::FileDescriptor() :
    _fd(-1)
{
}

FileDescriptor::FileDescriptor(int fd) :
    _fd(fd)
{
}

FileDescriptor::~FileDescriptor()
{
    close();
}

bool FileDescriptor::open_device(const char *path, int flags)
{
    if (path == nullptr || _fd >= 0) {
        return false;
    }

    _fd = ::open(path, flags | O_CLOEXEC);
    return _fd >= 0;
}

void FileDescriptor::attach_for_test(int fd)
{
    close();
    _fd = fd;
}

void FileDescriptor::close()
{
    if (_fd >= 0) {
        (void)::close(_fd);
        _fd = -1;
    }
}

bool FileDescriptor::is_open() const
{
    return _fd >= 0;
}

int FileDescriptor::native_fd() const
{
    return _fd;
}

ssize_t FileDescriptor::read_some(void *buffer, size_t size) const
{
    if (_fd < 0 || buffer == nullptr || size == 0U) {
        return -1;
    }

    ssize_t result;
    do {
        result = ::read(_fd, buffer, size);
    } while (result < 0 && errno == EINTR);
    return result;
}

bool FileDescriptor::write_all(const void *buffer, size_t size, uint8_t retry_limit) const
{
    if (_fd < 0 || (buffer == nullptr && size != 0U)) {
        return false;
    }

    const auto *bytes = static_cast<const uint8_t *>(buffer);
    size_t written = 0U;
    uint8_t retries = 0U;
    while (written < size) {
        const ssize_t result = ::write(_fd, bytes + written, size - written);
        if (result > 0) {
            written += static_cast<size_t>(result);
            retries = 0U;
            continue;
        }
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && retries < retry_limit) {
            retries++;
            continue;
        }
        return false;
    }
    return true;
}

StorageFile::StorageFile() :
    _fd(-1),
    _size(0U)
{
}

StorageFile::~StorageFile()
{
    close();
}

bool StorageFile::init(const char *path, size_t size)
{
    if (path == nullptr || size == 0U || _fd >= 0) {
        return false;
    }

    _fd = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (_fd < 0) {
#if defined(__NuttX__)
        if (strcmp(path, STORAGE_PATH) == 0) {
            storage_init_marker(errno == EROFS
                ? "SPRESENSE_M1_STORAGE=OPEN_EROFS\n"
                : errno == EACCES
                    ? "SPRESENSE_M1_STORAGE=OPEN_EACCES\n"
                    : "SPRESENSE_M1_STORAGE=OPEN_FAIL\n");
        }
#endif
        return false;
    }
#if defined(__NuttX__)
    if (strcmp(path, STORAGE_PATH) == 0) {
        storage_init_marker("SPRESENSE_M1_STORAGE=OPEN_OK\n");
    }
#endif
    if (ftruncate(_fd, static_cast<off_t>(size)) != 0) {
#if defined(__NuttX__)
        if (strcmp(path, STORAGE_PATH) == 0) {
            storage_init_marker(errno == ENOSPC
                ? "SPRESENSE_M1_STORAGE=TRUNCATE_ENOSPC\n"
                : errno == EROFS
                    ? "SPRESENSE_M1_STORAGE=TRUNCATE_EROFS\n"
                    : "SPRESENSE_M1_STORAGE=TRUNCATE_FAIL\n");
        }
#endif
        close();
        return false;
    }
    _size = size;
#if defined(__NuttX__)
    if (strcmp(path, STORAGE_PATH) == 0) {
        storage_init_marker("SPRESENSE_M1_STORAGE=READY\n");
    }
#endif
    return true;
}

void StorageFile::close()
{
    if (_fd >= 0) {
        (void)::close(_fd);
        _fd = -1;
    }
    _size = 0U;
}

bool StorageFile::healthy() const
{
    return _fd >= 0 && _size > 0U;
}

bool StorageFile::range_valid(size_t offset, size_t size) const
{
    return healthy() && offset <= _size && size <= _size - offset;
}

bool StorageFile::transfer(bool write_operation, size_t offset, void *buffer, size_t size)
{
    if (!range_valid(offset, size) || (buffer == nullptr && size != 0U)) {
        return false;
    }
    if (lseek(_fd, static_cast<off_t>(offset), SEEK_SET) < 0) {
        return false;
    }

    auto *bytes = static_cast<uint8_t *>(buffer);
    size_t transferred = 0U;
    while (transferred < size) {
        ssize_t result;
        if (write_operation) {
            result = ::write(_fd, bytes + transferred, size - transferred);
        } else {
            result = ::read(_fd, bytes + transferred, size - transferred);
        }
        if (result > 0) {
            transferred += static_cast<size_t>(result);
            continue;
        }
        if (result < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return !write_operation || fsync(_fd) == 0;
}

bool StorageFile::read_block(void *destination, size_t source, size_t size)
{
    return transfer(false, source, destination, size);
}

bool StorageFile::write_block(size_t destination, const void *source, size_t size)
{
    return transfer(true, destination, const_cast<void *>(source), size);
}

bool OutputGuard::request_arm()
{
    _arm_reject_count++;
    return false;
}

bool OutputGuard::request_write(uint8_t channel, uint16_t period_us)
{
    (void)channel;
    (void)period_us;
    _write_reject_count++;
    return false;
}

bool OutputGuard::armed() const
{
    return false;
}

bool OutputGuard::physical_output_enabled() const
{
    return false;
}

uint32_t OutputGuard::arm_reject_count() const
{
    return _arm_reject_count;
}

uint32_t OutputGuard::write_reject_count() const
{
    return _write_reject_count;
}

uint32_t OutputGuard::physical_write_count() const
{
    return 0U;
}

} // namespace Spresense
