#include "SensorBridge.h"

#include <math.h>

namespace {

constexpr float GRAVITY_M_S2 = 9.80665f;
constexpr uint16_t UNKNOWN_DOP = UINT16_MAX;
#if defined(__NuttX__)
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

#include <nuttx/config.h>

#include <arch/chip/gnss.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

int gnss_fd = -1;
int pwbimu_fd = -1;
bool gnss_have_timestamp;
uint64_t gnss_last_timestamp;

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

} // namespace

bool Spresense::sensor_bridge_platform_ready()
{
    return true;
}

bool Spresense::gnss_start()
{
    if (gnss_fd >= 0) {
        return true;
    }
    gnss_fd = open(CONFIG_SPRESENSE_M1_COPTER_GNSS_DEVICE,
                   O_RDONLY | O_NONBLOCK);
    if (gnss_fd < 0) {
        return false;
    }

    char version[CXD56_GNSS_VERSION_MAXLEN] {};
    if (checked_ioctl(gnss_fd, CXD56_GNSS_IOCTL_WAKEUP, 0U) != 0 ||
        checked_ioctl(gnss_fd, CXD56_GNSS_IOCTL_GET_VERSION,
                      reinterpret_cast<unsigned long>(version)) != 0 ||
        version[0] == '\0' ||
        checked_ioctl(gnss_fd, CXD56_GNSS_IOCTL_START,
                      CXD56_GNSS_STMOD_HOT) != 0) {
        close_device(gnss_fd);
        return false;
    }
    gnss_have_timestamp = false;
    return true;
}

Spresense::SensorReadStatus Spresense::gnss_read(GnssSample &sample)
{
    if (gnss_fd < 0 || !ready_to_read(gnss_fd, 0)) {
        return SensorReadStatus::NO_DATA;
    }

    struct cxd56_gnss_positiondata2_s position {};
    ssize_t length;
    do {
        length = read(gnss_fd, &position, sizeof(position));
    } while (length < 0 && errno == EINTR);
    if (length != static_cast<ssize_t>(sizeof(position))) {
        return SensorReadStatus::ERROR;
    }
    if (gnss_have_timestamp && position.timestamp == gnss_last_timestamp) {
        return SensorReadStatus::NO_DATA;
    }
    gnss_have_timestamp = true;
    gnss_last_timestamp = position.timestamp;

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
    const GnssRawSample raw {
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
    return convert_gnss_sample(raw, sample)
        ? SensorReadStatus::SAMPLE : SensorReadStatus::ERROR;
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
