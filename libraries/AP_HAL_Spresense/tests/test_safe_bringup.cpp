#include "../SafeBringup.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

namespace {

int failures;

void expect(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

void test_monotonic_clock()
{
    Spresense::MonotonicClock clock;
    uint64_t before = 0U;
    uint64_t after = 0U;
    expect(clock.micros(before), "read monotonic clock before delay");
    expect(clock.delay_microseconds(2000U), "bounded monotonic delay");
    expect(clock.micros(after), "read monotonic clock after delay");
    expect(after >= before + 1000U, "monotonic time advances");
}

void test_file_descriptor()
{
    int pipe_fds[2] {-1, -1};
    expect(pipe(pipe_fds) == 0, "create test pipe");
    if (pipe_fds[0] < 0 || pipe_fds[1] < 0) {
        return;
    }

    Spresense::FileDescriptor writer(pipe_fds[1]);
    const char payload[] = "spresense-console";
    expect(writer.write_all(payload, sizeof(payload)), "write complete console frame");

    char received[sizeof(payload)] {};
    expect(read(pipe_fds[0], received, sizeof(received)) == static_cast<ssize_t>(sizeof(received)),
           "read complete console frame");
    expect(memcmp(payload, received, sizeof(payload)) == 0, "console frame preserves bytes");
    (void)close(pipe_fds[0]);

    Spresense::FileDescriptor missing;
    expect(!missing.open_device("/definitely/not/a/spresense/device", O_RDWR),
           "missing UART fails without fallback");
    expect(!missing.is_open(), "failed UART remains closed");
}

void test_storage()
{
    char path[] = "/tmp/spresense-ardupilot-storage-XXXXXX";
    const int temporary_fd = mkstemp(path);
    expect(temporary_fd >= 0, "create storage fixture");
    if (temporary_fd < 0) {
        return;
    }
    (void)close(temporary_fd);

    Spresense::StorageFile storage;
    expect(storage.init(path, 64U), "initialize fixed storage file");
    const uint8_t expected[] {0x53U, 0x50U, 0x46U, 0x43U};
    uint8_t actual[sizeof(expected)] {};
    expect(storage.write_block(8U, expected, sizeof(expected)), "write bounded storage block");
    expect(storage.read_block(actual, 8U, sizeof(actual)), "read bounded storage block");
    expect(memcmp(expected, actual, sizeof(expected)) == 0, "storage preserves bytes");
    expect(!storage.write_block(63U, expected, sizeof(expected)), "reject storage overflow");
    storage.close();
    (void)unlink(path);

    Spresense::StorageFile no_fallback;
    expect(!no_fallback.init("/definitely/not/mounted/ardupilot.stg", 64U),
           "missing configured storage fails without fallback");
}

void test_output_guard()
{
    Spresense::OutputGuard guard;
    expect(!guard.request_arm(), "arming request is rejected");
    expect(!guard.request_write(0U, 1500U), "actuator write is rejected");
    expect(!guard.armed(), "arming state remains false");
    expect(!guard.physical_output_enabled(), "physical output remains disabled");
    expect(guard.arm_reject_count() == 1U, "arming rejection is counted");
    expect(guard.write_reject_count() == 1U, "write rejection is counted");
    expect(guard.physical_write_count() == 0U, "physical write count remains zero");
}

} // namespace

int main()
{
    test_monotonic_clock();
    test_file_descriptor();
    test_storage();
    test_output_guard();
    if (failures != 0) {
        fprintf(stderr, "spresense_m1_host=FAIL failures=%d\n", failures);
        return 1;
    }
    printf("spresense_m1_host=PASS outputs=disabled physical_writes=0\n");
    return 0;
}
