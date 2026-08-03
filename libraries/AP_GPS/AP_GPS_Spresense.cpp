#include "AP_GPS_config.h"

#if AP_GPS_SPRESENSE_ENABLED

#include "AP_GPS_Spresense.h"

#include <AP_HAL_Spresense/SensorBridge.h>
#include <AP_Math/AP_Math.h>

bool AP_GPS_Spresense::read()
{
    if (!Spresense::gnss_start()) {
        _healthy = false;
        return false;
    }

    Spresense::GnssSample sample {};
    const auto result = Spresense::gnss_read(sample);
    if (result == Spresense::SensorReadStatus::NO_DATA) {
        return false;
    }
    if (result == Spresense::SensorReadStatus::ERROR) {
        _healthy = false;
        return false;
    }

    _healthy = true;
    state.status = static_cast<AP_GPS::GPS_Status>(sample.fix_status);
    state.num_sats = sample.satellites_used;
    state.hdop = sample.hdop_x100;
    state.vdop = sample.vdop_x100;
    state.location.lat = sample.latitude_e7;
    state.location.lng = sample.longitude_e7;
    state.location.alt = sample.altitude_cm;
    state.ground_speed = sample.ground_speed_m_s;
    state.ground_course = wrap_360(sample.ground_course_deg);
    state.velocity = Vector3f(sample.velocity_north_m_s,
                              sample.velocity_east_m_s,
                              sample.velocity_down_m_s);
    state.have_vertical_velocity = sample.have_vertical_velocity;
    state.speed_accuracy = sample.speed_accuracy_m_s;
    state.have_speed_accuracy = sample.have_speed_accuracy;
    state.horizontal_accuracy = sample.horizontal_accuracy_m;
    state.have_horizontal_accuracy = sample.have_horizontal_accuracy;
    state.vertical_accuracy = sample.vertical_accuracy_m;
    state.have_vertical_accuracy = sample.have_vertical_accuracy;
    if (sample.year >= 2000U && sample.month >= 1U && sample.month <= 12U &&
        sample.day >= 1U && sample.day <= 31U && sample.hour <= 23U &&
        sample.minute <= 59U && sample.second <= 60U) {
        const uint32_t date = static_cast<uint32_t>(sample.day) * 10000U +
            static_cast<uint32_t>(sample.month) * 100U +
            sample.year % 100U;
        const uint32_t time_ms =
            (((static_cast<uint32_t>(sample.hour) * 100U + sample.minute) *
              100U + sample.second) * 1000U) +
            sample.microsecond / 1000U;
        make_gps_time(date, time_ms);
    }
    return true;
}

bool AP_GPS_Spresense::get_lag(float &lag) const
{
    lag = 0.2f;
    return false;
}

#endif // AP_GPS_SPRESENSE_ENABLED
