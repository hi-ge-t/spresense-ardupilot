#include "SensorBridge.h"

#if defined(__NuttX__)
#include <nuttx/config.h>
#endif

#include <math.h>

namespace {

constexpr float GRAVITY_M_S2 = 9.80665f;
constexpr uint16_t UNKNOWN_DOP = UINT16_MAX;
#if defined(__NuttX__)
constexpr int GNSS_READER_POLL_TIMEOUT_MS = 1000;
constexpr uint32_t GNSS_READER_YIELD_US = 1000U;
constexpr int PWBIMU_POLL_TIMEOUT_MS = 1;
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
#include <stdint.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

namespace {

int gnss_fd = -1;
int pwbimu_fd = -1;
bool gnss_have_timestamp;
uint64_t gnss_last_timestamp;
uint8_t gnss_init_state;
pthread_mutex_t gnss_sample_mutex = PTHREAD_MUTEX_INITIALIZER;
Spresense::GnssSample gnss_latest_sample {};
uint32_t gnss_sample_sequence;
uint32_t gnss_consumed_sequence;
constexpr uint8_t GNSS_INIT_IDLE = 0U;
constexpr uint8_t GNSS_INIT_STARTING = 1U;
constexpr uint8_t GNSS_INIT_READY = 2U;
constexpr uint8_t GNSS_INIT_FAILED = 3U;
constexpr int GNSS_INIT_PRIORITY = 110;
// The reader keeps Sony's 1328-byte PVT structure on its stack and enters
// driver/libc calls below it.  Match the other Spresense worker stacks rather
// than relying on the NuttX minimum with too little diagnostic margin.
constexpr size_t GNSS_INIT_STACK_BYTES = 8192U;

int checked_ioctl(int fd, int request, unsigned long argument)
{
    if (ioctl(fd, request, argument) < 0) {
        return errno == 0 ? -EIO : -errno;
    }
    return 0;
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

void gnss_init_fail(int &fd)
{
    close_device(fd);
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
        if (!ready_to_read(gnss_fd, GNSS_READER_POLL_TIMEOUT_MS)) {
            continue;
        }

        struct cxd56_gnss_positiondata2_s position {};
        ssize_t length;
        do {
            length = read(gnss_fd, &position, sizeof(position));
        } while (length < 0 && errno == EINTR);
        if (length != static_cast<ssize_t>(sizeof(position)) ||
            !publish_gnss_sample(position)) {
            gnss_init_fail(gnss_fd);
            return;
        }

        // The CXD5610 poll notification is level-like on this SDK.  Block
        // briefly after consuming a snapshot so an immediately reasserted
        // notification cannot monopolize the single application core.
        struct timespec yield_time {0, GNSS_READER_YIELD_US * 1000L};
        while (nanosleep(&yield_time, &yield_time) != 0 && errno == EINTR) {
        }
    }
}

void *gnss_init_thread(void *)
{
    int fd = open(CONFIG_SPRESENSE_M1_COPTER_GNSS_DEVICE,
                  O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        gnss_init_fail(fd);
        return nullptr;
    }

    char version[CXD56_GNSS_VERSION_MAXLEN] {};
    if (checked_ioctl(fd, CXD56_GNSS_IOCTL_WAKEUP, 0U) != 0) {
        gnss_init_fail(fd);
        return nullptr;
    }
    if (checked_ioctl(fd, CXD56_GNSS_IOCTL_GET_VERSION,
                      reinterpret_cast<unsigned long>(version)) != 0 ||
        version[0] == '\0') {
        gnss_init_fail(fd);
        return nullptr;
    }
    if (checked_ioctl(fd, CXD56_GNSS_IOCTL_START,
                      CXD56_GNSS_STMOD_HOT) != 0) {
        gnss_init_fail(fd);
        return nullptr;
    }

    gnss_fd = fd;
    gnss_have_timestamp = false;
    __atomic_store_n(&gnss_init_state, GNSS_INIT_READY, __ATOMIC_RELEASE);
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
    return true;
}

bool Spresense::gnss_start()
{
    // Sony's start ioctls synchronously wait for responses produced by the
    // CXD5610 receive task.  Run that bounded handshake away from Copter's
    // main loop so a missing or failed Add-on cannot stop GCS and INS work.
    const uint8_t state = __atomic_load_n(
        &gnss_init_state, __ATOMIC_ACQUIRE);
    if (state == GNSS_INIT_READY) {
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
    return true;
}

Spresense::SensorReadStatus Spresense::pwbimu_read(ImuSample &sample)
{
    if (pwbimu_fd < 0) {
        return SensorReadStatus::ERROR;
    }

    // During Copter setup the scheduler's timer processes are intentionally
    // held until setup() returns.  A bounded poll yields the high-priority
    // main task so Sony's SPI5/DMAC driver can publish its first sample;
    // nonblocking reads alone can otherwise starve that producer.  This is a
    // readiness bound, not evidence for the eventual sensor-loop timing.
    if (!ready_to_read(pwbimu_fd, PWBIMU_POLL_TIMEOUT_MS)) {
        return SensorReadStatus::NO_DATA;
    }

    cxd5602pwbimu_data_t data {};
    ssize_t length;
    do {
        length = read(pwbimu_fd, &data, sizeof(data));
    } while (length < 0 && errno == EINTR);
    if (length < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return SensorReadStatus::NO_DATA;
    }
    if (length != static_cast<ssize_t>(sizeof(data))) {
        return SensorReadStatus::ERROR;
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
