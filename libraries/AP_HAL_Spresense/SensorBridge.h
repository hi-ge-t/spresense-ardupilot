#pragma once

#include <stdint.h>

namespace Spresense {

enum class SensorReadStatus : int8_t {
    ERROR = -1,
    NO_DATA = 0,
    SAMPLE = 1,
};

enum class GnssFixStatus : uint8_t {
    NO_FIX = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_3D_DGPS = 4,
};

struct GnssRawSample {
    uint64_t timestamp_us;
    uint32_t device_status;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t microsecond;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    float ground_speed_m_s;
    float ground_course_deg;
    float up_velocity_m_s;
    float horizontal_accuracy_m;
    float vertical_accuracy_m;
    float horizontal_speed_accuracy_km_h;
    float hdop;
    float vdop;
    uint8_t fix_indicator;
    uint8_t position_fix_mode;
    uint8_t velocity_fix_mode;
    uint8_t satellites_used;
    bool position_data_exists;
};

struct GnssSample {
    uint64_t timestamp_us;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t microsecond;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_cm;
    float ground_speed_m_s;
    float ground_course_deg;
    float velocity_north_m_s;
    float velocity_east_m_s;
    float velocity_down_m_s;
    float speed_accuracy_m_s;
    float horizontal_accuracy_m;
    float vertical_accuracy_m;
    uint16_t hdop_x100;
    uint16_t vdop_x100;
    uint8_t satellites_used;
    GnssFixStatus fix_status;
    bool have_vertical_velocity;
    bool have_speed_accuracy;
    bool have_horizontal_accuracy;
    bool have_vertical_accuracy;
};

struct ImuRawSample {
    uint32_t timestamp_ticks;
    float temperature_c;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
};

struct ImuSample {
    uint32_t timestamp_ticks;
    float temperature_c;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
    float accel_x_m_s2;
    float accel_y_m_s2;
    float accel_z_m_s2;
};

bool convert_gnss_sample(const GnssRawSample &raw, GnssSample &sample);
bool convert_imu_sample(const ImuRawSample &raw, ImuSample &sample);

bool sensor_bridge_platform_ready();
bool gnss_start();
SensorReadStatus gnss_read(GnssSample &sample);
bool pwbimu_start(uint16_t sample_rate_hz);
SensorReadStatus pwbimu_read(ImuSample &sample);

} // namespace Spresense
