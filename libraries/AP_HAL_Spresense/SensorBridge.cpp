#include "SensorBridge.h"

#if defined(__NuttX__)
#include <nuttx/config.h>
#endif

#include <math.h>

namespace {

constexpr float GRAVITY_M_S2 = 9.80665f;
constexpr uint16_t UNKNOWN_DOP = UINT16_MAX;
#if defined(__NuttX__)
constexpr int GNSS_NOTIFICATION_SIGNAL = 18;
constexpr uint32_t GNSS_NOTIFICATION_WAIT_MS = 1250U;
constexpr uint32_t GNSS_READER_YIELD_US = 1000U;
constexpr int PWBIMU_STARTUP_POLL_TIMEOUT_MS = 1000;
#endif

bool finite_position(const Spresense::GnssRawSample &raw)
{
    return isfinite(raw.latitude_deg) &&
           isfinite(raw.longitude_deg) &&
           isfinite(raw.altitude_m) &&
           raw.latitude_deg >= -90.0 && raw.latitude_deg <= 90.0 &&
           raw.longitude_deg >= -180.0 && raw.longitude_deg <= 180.0;
}

uint16_t scale_dop(float value)
{
    if (!isfinite(value) || value < 0.0f) {
        return UNKNOWN_DOP;
    }
    const float scaled = value * 100.0f;
    if (scaled >= static_cast<float>(UNKNOWN_DOP)) {
        return UNKNOWN_DOP - 1U;
    }
    return static_cast<uint16_t>(lroundf(scaled));
}

Spresense::GnssFixStatus fix_status(const Spresense::GnssRawSample &raw)
{
    if (raw.device_status != 0U || !raw.position_data_exists ||
        raw.fix_indicator == 0U || !finite_position(raw)) {
        return Spresense::GnssFixStatus::NO_FIX;
    }
    if (raw.position_fix_mode == 2U) {
        return Spresense::GnssFixStatus::FIX_2D;
    }
    if (raw.position_fix_mode == 3U) {
        return raw.fix_indicator == 2U
            ? Spresense::GnssFixStatus::FIX_3D_DGPS
            : Spresense::GnssFixStatus::FIX_3D;
    }
    return Spresense::GnssFixStatus::NO_FIX;
}

} // namespace

bool Spresense::convert_gnss_sample(const GnssRawSample &raw,
                                    GnssSample &sample)
{
    sample = {};
    sample.timestamp_us = raw.timestamp_us;
    sample.year = raw.year;
    sample.month = raw.month;
    sample.day = raw.day;
    sample.hour = raw.hour;
    sample.minute = raw.minute;
    sample.second = raw.second;
    sample.microsecond = raw.microsecond;
    sample.fix_status = fix_status(raw);
    sample.satellites_used = raw.satellites_used;
    sample.hdop_x100 = scale_dop(raw.hdop);
    sample.vdop_x100 = scale_dop(raw.vdop);

    if (sample.fix_status == GnssFixStatus::NO_FIX) {
        return true;
    }

    sample.latitude_e7 = static_cast<int32_t>(
        llround(raw.latitude_deg * 1.0e7));
    sample.longitude_e7 = static_cast<int32_t>(
        llround(raw.longitude_deg * 1.0e7));
    sample.altitude_cm = static_cast<int32_t>(
        llround(raw.altitude_m * 100.0));

    if (isfinite(raw.ground_speed_m_s) &&
        isfinite(raw.ground_course_deg)) {
        sample.ground_speed_m_s = raw.ground_speed_m_s;
        sample.ground_course_deg = raw.ground_course_deg;
        const float course_rad = raw.ground_course_deg *
            (3.14159265358979323846f / 180.0f);
        sample.velocity_north_m_s = raw.ground_speed_m_s *
            cosf(course_rad);
        sample.velocity_east_m_s = raw.ground_speed_m_s *
            sinf(course_rad);
    }
    if (raw.velocity_fix_mode != 0U && raw.velocity_fix_mode != 1U &&
        isfinite(raw.up_velocity_m_s)) {
        sample.velocity_down_m_s = -raw.up_velocity_m_s;
        sample.have_vertical_velocity = true;
    }
    if (isfinite(raw.horizontal_speed_accuracy_km_h) &&
        raw.horizontal_speed_accuracy_km_h >= 0.0f) {
        sample.speed_accuracy_m_s =
            raw.horizontal_speed_accuracy_km_h / 3.6f;
        sample.have_speed_accuracy = true;
    }
    if (isfinite(raw.horizontal_accuracy_m) &&
        raw.horizontal_accuracy_m >= 0.0f) {
        sample.horizontal_accuracy_m = raw.horizontal_accuracy_m;
        sample.have_horizontal_accuracy = true;
    }
    if (isfinite(raw.vertical_accuracy_m) &&
        raw.vertical_accuracy_m >= 0.0f) {
        sample.vertical_accuracy_m = raw.vertical_accuracy_m;
        sample.have_vertical_accuracy = true;
    }
    return true;
}

bool Spresense::convert_imu_sample(const ImuRawSample &raw,
                                   ImuSample &sample)
{
    if (!isfinite(raw.temperature_c) ||
        !isfinite(raw.gyro_x_rad_s) ||
        !isfinite(raw.gyro_y_rad_s) ||
        !isfinite(raw.gyro_z_rad_s) ||
        !isfinite(raw.accel_x_g) ||
        !isfinite(raw.accel_y_g) ||
        !isfinite(raw.accel_z_g)) {
        return false;
    }
    sample.timestamp_ticks = raw.timestamp_ticks;
    sample.temperature_c = raw.temperature_c;
    sample.gyro_x_rad_s = raw.gyro_x_rad_s;
    sample.gyro_y_rad_s = raw.gyro_y_rad_s;
    sample.gyro_z_rad_s = raw.gyro_z_rad_s;
    sample.accel_x_m_s2 = raw.accel_x_g * GRAVITY_M_S2;
    sample.accel_y_m_s2 = raw.accel_y_g * GRAVITY_M_S2;
    sample.accel_z_m_s2 = raw.accel_z_g * GRAVITY_M_S2;
    return true;
}

#if defined(__NuttX__)

#include <arch/chip/gnss.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

namespace {

int gnss_fd = -1;
int pwbimu_fd = -1;
bool gnss_have_timestamp;
uint64_t gnss_last_timestamp;
uint8_t gnss_init_state;
bool gnss_signal_configured;
pthread_mutex_t gnss_sample_mutex = PTHREAD_MUTEX_INITIALIZER;
Spresense::GnssSample gnss_latest_sample {};
struct cxd56_gnss_positiondata2_s gnss_position {};
uint32_t gnss_sample_sequence;
uint32_t gnss_consumed_sequence;
bool gnss_timeout_reported;
bool gnss_notification_reported;
bool gnss_sample_reported;
bool gnss_attach_reported;
bool gnss_consumed_reported;
bool pwbimu_read_after_gnss_reported;
bool pwbimu_return_after_gnss_reported;
bool pwbimu_sample_after_gnss_reported;
bool pwbimu_eagain_after_gnss_reported;
bool pwbimu_first_empty_after_gnss_reported;
bool pwbimu_empty_count_after_gnss_reported;
uint32_t pwbimu_eagain_after_gnss_count;
uint32_t pwbimu_empty_after_gnss_count;
constexpr uint8_t GNSS_INIT_IDLE = 0U;
constexpr uint8_t GNSS_INIT_STARTING = 1U;
constexpr uint8_t GNSS_INIT_READY = 2U;
constexpr uint8_t GNSS_INIT_FAILED = 3U;
constexpr int GNSS_INIT_PRIORITY = 110;
// Match the other Spresense worker stacks.  Sony's 1328-byte PVT snapshot is
// static, as in the hardware-verified bounded GCS probe, rather than local.
constexpr size_t GNSS_INIT_STACK_BYTES = 8192U;

enum class GnssWaitStatus : uint8_t {
    NOTIFIED,
    TIMEOUT,
    ERROR,
};

int checked_ioctl(int fd, int request, unsigned long argument)
{
    if (ioctl(fd, request, argument) < 0) {
        return errno == 0 ? -EIO : -errno;
    }
    return 0;
}

bool gnss_notification_mask(sigset_t &mask)
{
    return sigemptyset(&mask) == 0 &&
           sigaddset(&mask, GNSS_NOTIFICATION_SIGNAL) == 0;
}

bool block_gnss_notification()
{
    sigset_t mask {};
    return gnss_notification_mask(mask) &&
           pthread_sigmask(SIG_BLOCK, &mask, nullptr) == 0;
}

bool configure_gnss_notification(int fd, bool enable)
{
    struct cxd56_gnss_signal_setting_s setting {};
    setting.fd = fd;
    setting.enable = enable;
    setting.gnsssig = CXD56_GNSS_SIG_GNSS;
    setting.signo = GNSS_NOTIFICATION_SIGNAL;
    return checked_ioctl(
        fd, CXD56_GNSS_IOCTL_SIGNAL_SET,
        reinterpret_cast<unsigned long>(&setting)) == 0;
}

GnssWaitStatus wait_gnss_notification()
{
    sigset_t mask {};
    if (!gnss_notification_mask(mask)) {
        return GnssWaitStatus::ERROR;
    }
    const struct timespec timeout {
        static_cast<time_t>(GNSS_NOTIFICATION_WAIT_MS / 1000U),
        static_cast<long>(GNSS_NOTIFICATION_WAIT_MS % 1000U) * 1000000L,
    };
    int result;
    do {
        result = sigtimedwait(&mask, nullptr, &timeout);
    } while (result < 0 && errno == EINTR);
    if (result == GNSS_NOTIFICATION_SIGNAL) {
        return GnssWaitStatus::NOTIFIED;
    }
    if (result < 0 && errno == EAGAIN) {
        return GnssWaitStatus::TIMEOUT;
    }
    return GnssWaitStatus::ERROR;
}

bool ready_to_read(int fd, int timeout_ms)
{
    struct pollfd descriptor {fd, POLLIN, 0};
    int result;
    do {
        result = poll(&descriptor, 1, timeout_ms);
    } while (result < 0 && errno == EINTR);
    return result > 0 && (descriptor.revents & POLLIN) != 0;
}

void close_device(int &fd)
{
    if (fd >= 0) {
        (void)close(fd);
        fd = -1;
    }
}

void gnss_marker(const char *marker)
{
    (void)write(STDOUT_FILENO, marker, strlen(marker));
}

void gnss_init_fail(int &fd, const char *marker)
{
    if (fd >= 0 && gnss_signal_configured) {
        (void)configure_gnss_notification(fd, false);
        gnss_signal_configured = false;
    }
    close_device(fd);
    gnss_marker(marker);
    __atomic_store_n(&gnss_init_state, GNSS_INIT_FAILED, __ATOMIC_RELEASE);
}

bool publish_gnss_sample(const struct cxd56_gnss_positiondata2_s &position)
{
    if (gnss_have_timestamp && position.timestamp == gnss_last_timestamp) {
        return true;
    }

    uint8_t satellites_used = 0U;
    const uint32_t count = position.svcount < CXD56_GNSS_MAX_SV2_NUM
        ? position.svcount : CXD56_GNSS_MAX_SV2_NUM;
    for (uint32_t index = 0U; index < count; index++) {
        if ((position.sv[index].stat & (1U << 1U)) != 0U &&
            satellites_used != UINT8_MAX) {
            satellites_used++;
        }
    }

    const auto &receiver = position.receiver;
    const Spresense::GnssRawSample raw {
        position.timestamp,
        position.status,
        receiver.date.year,
        receiver.date.month,
        receiver.date.day,
        receiver.time.hour,
        receiver.time.minute,
        receiver.time.sec,
        receiver.time.usec,
        receiver.latitude,
        receiver.longitude,
        receiver.altitude,
        receiver.velocity,
        receiver.direction,
        receiver.up_velocity,
        receiver.hvar,
        receiver.vvar,
        receiver.hvar_speed,
        receiver.hdop,
        receiver.vdop,
        receiver.fix_indicator,
        receiver.pos_fixmode,
        receiver.vel_fixmode,
        satellites_used,
        receiver.pos_dataexist != 0U,
    };
    Spresense::GnssSample sample {};
    if (!Spresense::convert_gnss_sample(raw, sample) ||
        pthread_mutex_lock(&gnss_sample_mutex) != 0) {
        return false;
    }
    gnss_latest_sample = sample;
    gnss_sample_sequence++;
    if (pthread_mutex_unlock(&gnss_sample_mutex) != 0) {
        return false;
    }
    gnss_have_timestamp = true;
    gnss_last_timestamp = position.timestamp;
    return true;
}

void run_gnss_reader()
{
    for (;;) {
        const GnssWaitStatus wait_status = wait_gnss_notification();
        if (wait_status == GnssWaitStatus::TIMEOUT) {
            if (!gnss_timeout_reported) {
                gnss_timeout_reported = true;
                gnss_marker("SPRESENSE_M1_GNSS=WAIT_TIMEOUT\n");
            }
            continue;
        }
        if (wait_status != GnssWaitStatus::NOTIFIED) {
            gnss_init_fail(gnss_fd, "SPRESENSE_M1_GNSS=WAIT_FAIL\n");
            return;
        }
        if (!gnss_notification_reported) {
            gnss_notification_reported = true;
            gnss_marker("SPRESENSE_M1_GNSS=NOTIFIED\n");
        }

        gnss_position = {};
        ssize_t length;
        do {
            length = read(gnss_fd, &gnss_position, sizeof(gnss_position));
        } while (length < 0 && errno == EINTR);
        if (length != static_cast<ssize_t>(sizeof(gnss_position)) ||
            !publish_gnss_sample(gnss_position)) {
            gnss_init_fail(gnss_fd, "SPRESENSE_M1_GNSS=STREAM_FAIL\n");
            return;
        }
        if (!gnss_sample_reported) {
            gnss_sample_reported = true;
            gnss_marker("SPRESENSE_M1_GNSS=SAMPLE\n");
            gnss_marker(sched_lockcount() == 0
                ? "SPRESENSE_M1_GNSS=SAMPLE_LOCK_0\n"
                : "SPRESENSE_M1_GNSS=SAMPLE_LOCK_NONZERO\n");
        }

        // The Add-on notification behaves like a level signal on this SDK.
        // Yield after consuming a snapshot so an immediately reasserted
        // signal cannot starve the PWBIMU producer on the application core.
        // This is a scheduling guard, not a claim about flight-loop timing.
        struct timespec yield_time {
            0, static_cast<long>(GNSS_READER_YIELD_US) * 1000L
        };
        while (nanosleep(&yield_time, &yield_time) != 0 && errno == EINTR) {
        }
    }
}

void *gnss_init_thread(void *)
{
    gnss_marker("SPRESENSE_M1_GNSS=THREAD\n");
    int fd = open(CONFIG_SPRESENSE_M1_COPTER_GNSS_DEVICE,
                  O_RDONLY);
    if (fd < 0) {
        gnss_init_fail(fd, "SPRESENSE_M1_GNSS=OPEN_FAIL\n");
        return nullptr;
    }
    gnss_marker("SPRESENSE_M1_GNSS=OPEN\n");

    char version[CXD56_GNSS_VERSION_MAXLEN] {};
    if (checked_ioctl(fd, CXD56_GNSS_IOCTL_WAKEUP, 0U) != 0) {
        gnss_init_fail(fd, "SPRESENSE_M1_GNSS=WAKE_FAIL\n");
        return nullptr;
    }
    gnss_marker("SPRESENSE_M1_GNSS=WAKE\n");
    if (checked_ioctl(fd, CXD56_GNSS_IOCTL_GET_VERSION,
                      reinterpret_cast<unsigned long>(version)) != 0 ||
        version[0] == '\0') {
        gnss_init_fail(fd, "SPRESENSE_M1_GNSS=VERSION_FAIL\n");
        return nullptr;
    }
    gnss_marker("SPRESENSE_M1_GNSS=VERSION\n");
    gnss_marker("SPRESENSE_M1_GNSS=SIGNAL_BEGIN\n");
    if (!block_gnss_notification() ||
        !configure_gnss_notification(fd, true)) {
        gnss_init_fail(fd, "SPRESENSE_M1_GNSS=SIGNAL_FAIL\n");
        return nullptr;
    }
    gnss_signal_configured = true;
    gnss_marker("SPRESENSE_M1_GNSS=SIGNAL_CONFIGURED\n");
    if (checked_ioctl(fd, CXD56_GNSS_IOCTL_START,
                      CXD56_GNSS_STMOD_HOT) != 0) {
        gnss_init_fail(fd, "SPRESENSE_M1_GNSS=START_FAIL\n");
        return nullptr;
    }
    gnss_marker("SPRESENSE_M1_GNSS=START\n");
    gnss_marker(sched_lockcount() == 0
        ? "SPRESENSE_M1_GNSS=START_LOCK_0\n"
        : "SPRESENSE_M1_GNSS=START_LOCK_NONZERO\n");

    gnss_fd = fd;
    gnss_have_timestamp = false;
    __atomic_store_n(&gnss_init_state, GNSS_INIT_READY, __ATOMIC_RELEASE);
    gnss_marker("SPRESENSE_M1_GNSS=READY\n");
    run_gnss_reader();
    return nullptr;
}

bool start_gnss_init_thread()
{
    pthread_attr_t attributes {};
    if (pthread_attr_init(&attributes) != 0) {
        return false;
    }

    struct sched_param scheduling {};
    scheduling.sched_priority = GNSS_INIT_PRIORITY;
    const size_t stack_size = GNSS_INIT_STACK_BYTES < PTHREAD_STACK_MIN
        ? PTHREAD_STACK_MIN : GNSS_INIT_STACK_BYTES;
    const bool configured =
        pthread_attr_setstacksize(&attributes, stack_size) == 0 &&
        pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED) == 0 &&
        pthread_attr_setschedpolicy(&attributes, SCHED_FIFO) == 0 &&
        pthread_attr_setschedparam(&attributes, &scheduling) == 0 &&
        pthread_attr_setinheritsched(&attributes, PTHREAD_EXPLICIT_SCHED) == 0;
    pthread_t thread {};
    const int result = configured
        ? pthread_create(&thread, &attributes, gnss_init_thread, nullptr)
        : -1;
    (void)pthread_attr_destroy(&attributes);
    if (result == 0) {
        (void)pthread_setname_np(thread, "ap-gnss-init");
    }
    return result == 0;
}

} // namespace

bool Spresense::sensor_bridge_platform_ready()
{
    // The CXD5610 driver records getpid() when SIGNAL_SET is issued, so its
    // notification targets this pthread group rather than the reader pthread
    // alone.  Block the signal before Scheduler::init() creates child threads;
    // they inherit the mask and the GNSS reader becomes the sole consumer via
    // sigtimedwait().  Otherwise notifications repeatedly interrupt Copter's
    // sub-tick nanosleep and prevent wait_for_sample() from progressing.
    return block_gnss_notification();
}

bool Spresense::gnss_start()
{
    // Sony's start ioctls synchronously wait for responses produced by the
    // CXD5610 receive task.  Run that bounded handshake away from Copter's
    // main loop so a missing or failed Add-on cannot stop GCS and INS work.
    const uint8_t state = __atomic_load_n(
        &gnss_init_state, __ATOMIC_ACQUIRE);
    if (state == GNSS_INIT_READY) {
        if (!gnss_attach_reported) {
            gnss_attach_reported = true;
            gnss_marker("SPRESENSE_M1_GNSS=ATTACH\n");
        }
        return gnss_fd >= 0;
    }
    if (state == GNSS_INIT_STARTING || state == GNSS_INIT_FAILED) {
        return false;
    }

    uint8_t expected = GNSS_INIT_IDLE;
    if (!__atomic_compare_exchange_n(
            &gnss_init_state, &expected, GNSS_INIT_STARTING, false,
            __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        return expected == GNSS_INIT_READY && gnss_fd >= 0;
    }
    if (!start_gnss_init_thread()) {
        gnss_marker("SPRESENSE_M1_GNSS=THREAD_FAIL\n");
        __atomic_store_n(
            &gnss_init_state, GNSS_INIT_FAILED, __ATOMIC_RELEASE);
        return false;
    }
    return false;
}

Spresense::SensorReadStatus Spresense::gnss_read(GnssSample &sample)
{
    // The low-priority init thread becomes the blocking GNSS reader after the
    // handshake.  Copter only takes a completed snapshot with trylock, so a
    // missing notification or stalled driver cannot stop its main loop.
    const uint8_t state = __atomic_load_n(
        &gnss_init_state, __ATOMIC_ACQUIRE);
    if (state == GNSS_INIT_FAILED) {
        return SensorReadStatus::ERROR;
    }
    if (state != GNSS_INIT_READY ||
        pthread_mutex_trylock(&gnss_sample_mutex) != 0) {
        return SensorReadStatus::NO_DATA;
    }
    if (gnss_sample_sequence == gnss_consumed_sequence) {
        (void)pthread_mutex_unlock(&gnss_sample_mutex);
        return SensorReadStatus::NO_DATA;
    }
    sample = gnss_latest_sample;
    gnss_consumed_sequence = gnss_sample_sequence;
    if (pthread_mutex_unlock(&gnss_sample_mutex) != 0) {
        return SensorReadStatus::ERROR;
    }
    if (!gnss_consumed_reported) {
        gnss_consumed_reported = true;
        gnss_marker("SPRESENSE_M1_GNSS=CONSUMED\n");
    }
    return SensorReadStatus::SAMPLE;
}

bool Spresense::pwbimu_start(uint16_t sample_rate_hz)
{
    if (pwbimu_fd >= 0) {
        return true;
    }
    pwbimu_fd = open(CONFIG_SPRESENSE_M1_COPTER_PWBIMU_DEVICE,
                     O_RDONLY | O_NONBLOCK);
    if (pwbimu_fd < 0) {
        return false;
    }

    cxd5602pwbimu_range_t range {2, 125};
    if (checked_ioctl(pwbimu_fd, SNIOC_SSAMPRATE, sample_rate_hz) != 0 ||
        checked_ioctl(pwbimu_fd, SNIOC_SDRANGE,
                      reinterpret_cast<unsigned long>(&range)) != 0 ||
        checked_ioctl(pwbimu_fd, SNIOC_SFIFOTHRESH, 1U) != 0 ||
        checked_ioctl(pwbimu_fd, SNIOC_ENABLE, 1U) != 0) {
        close_device(pwbimu_fd);
        return false;
    }

    // Yield once after enabling the device so Sony's HPWORK producer can
    // publish the initial sample during Copter setup.  Do not use poll() in
    // the steady-state AP_InertialSensor path: the driver registers a single
    // poll waiter while taking its device lock, and repeated setup/teardown
    // from wait_for_sample() can block the flight-control thread.  The file
    // remains O_NONBLOCK, so a direct read below returns EAGAIN immediately.
    (void)ready_to_read(pwbimu_fd, PWBIMU_STARTUP_POLL_TIMEOUT_MS);
    return true;
}

Spresense::SensorReadStatus Spresense::pwbimu_read(ImuSample &sample)
{
    if (pwbimu_fd < 0) {
        return SensorReadStatus::ERROR;
    }

    const bool gnss_started = __atomic_load_n(
        &gnss_init_state, __ATOMIC_ACQUIRE) != GNSS_INIT_IDLE;
    if (gnss_started && !pwbimu_read_after_gnss_reported) {
        pwbimu_read_after_gnss_reported = true;
        gnss_marker("SPRESENSE_M1_PWBIMU=READ_AFTER_GNSS\n");
    }

    cxd5602pwbimu_data_t data {};
    ssize_t length;
    do {
        length = read(pwbimu_fd, &data, sizeof(data));
    } while (length < 0 && errno == EINTR);
    if (gnss_started && !pwbimu_return_after_gnss_reported) {
        pwbimu_return_after_gnss_reported = true;
        gnss_marker("SPRESENSE_M1_PWBIMU=READ_RETURNED\n");
    }
    if (gnss_started && length != static_cast<ssize_t>(sizeof(data))) {
        if (!pwbimu_first_empty_after_gnss_reported) {
            pwbimu_first_empty_after_gnss_reported = true;
            if (length < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                gnss_marker("SPRESENSE_M1_PWBIMU=FIRST_EAGAIN\n");
            } else if (length == 0) {
                gnss_marker("SPRESENSE_M1_PWBIMU=FIRST_ZERO\n");
            } else {
                gnss_marker("SPRESENSE_M1_PWBIMU=FIRST_SHORT_OR_ERROR\n");
            }
        }
        if (++pwbimu_empty_after_gnss_count >= 100U &&
            !pwbimu_empty_count_after_gnss_reported) {
            pwbimu_empty_count_after_gnss_reported = true;
            gnss_marker("SPRESENSE_M1_PWBIMU=EMPTY_100\n");
        }
    }
    if (length < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        if (gnss_started && ++pwbimu_eagain_after_gnss_count >= 1000U &&
            !pwbimu_eagain_after_gnss_reported) {
            pwbimu_eagain_after_gnss_reported = true;
            gnss_marker("SPRESENSE_M1_PWBIMU=EAGAIN_1000\n");
        }
        return SensorReadStatus::NO_DATA;
    }
    if (length != static_cast<ssize_t>(sizeof(data))) {
        return SensorReadStatus::ERROR;
    }
    if (gnss_started && !pwbimu_sample_after_gnss_reported) {
        pwbimu_sample_after_gnss_reported = true;
        gnss_marker("SPRESENSE_M1_PWBIMU=SAMPLE_AFTER_GNSS\n");
    }

    // Sony's PWBIMU stream supplies gyro in rad/s and acceleration in g.
    // The SDK posture examples consume gyro directly and normalize gravity
    // around 1 g; AP_InertialSensor requires rad/s and m/s^2.
    const ImuRawSample raw {
        data.timestamp,
        data.temp,
        data.gx,
        data.gy,
        data.gz,
        data.ax,
        data.ay,
        data.az,
    };
    return convert_imu_sample(raw, sample)
        ? SensorReadStatus::SAMPLE : SensorReadStatus::ERROR;
}

#else

bool Spresense::sensor_bridge_platform_ready()
{
    return false;
}

bool Spresense::gnss_start()
{
    return false;
}

Spresense::SensorReadStatus Spresense::gnss_read(GnssSample &)
{
    return SensorReadStatus::ERROR;
}

bool Spresense::pwbimu_start(uint16_t)
{
    return false;
}

Spresense::SensorReadStatus Spresense::pwbimu_read(ImuSample &)
{
    return SensorReadStatus::ERROR;
}

#endif
