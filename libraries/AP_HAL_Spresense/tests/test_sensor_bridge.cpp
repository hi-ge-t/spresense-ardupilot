#include "../SensorBridge.h"

#include <assert.h>
#include <cmath>
#include <limits>

namespace {

bool near(float lhs, float rhs, float tolerance = 1.0e-4f)
{
    return std::fabs(lhs - rhs) <= tolerance;
}

Spresense::GnssRawSample valid_gnss()
{
    return {
        123456U,
        0U,
        2026U,
        8U,
        4U,
        12U,
        34U,
        56U,
        789000U,
        35.681236,
        139.767125,
        42.5,
        5.0f,
        90.0f,
        1.25f,
        0.8f,
        1.2f,
        3.6f,
        0.75f,
        1.25f,
        2U,
        3U,
        4U,
        12U,
        true,
    };
}

} // namespace

int main()
{
    auto raw = valid_gnss();
    Spresense::GnssSample gnss {};
    assert(Spresense::convert_gnss_sample(raw, gnss));
    assert(gnss.fix_status == Spresense::GnssFixStatus::FIX_3D_DGPS);
    assert(gnss.latitude_e7 == 356812360);
    assert(gnss.longitude_e7 == 1397671250);
    assert(gnss.altitude_cm == 4250);
    assert(gnss.satellites_used == 12U);
    assert(gnss.hdop_x100 == 75U);
    assert(gnss.vdop_x100 == 125U);
    assert(near(gnss.velocity_north_m_s, 0.0f));
    assert(near(gnss.velocity_east_m_s, 5.0f));
    assert(near(gnss.velocity_down_m_s, -1.25f));
    assert(near(gnss.speed_accuracy_m_s, 1.0f));
    assert(gnss.have_vertical_velocity);
    assert(gnss.have_speed_accuracy);
    assert(gnss.have_horizontal_accuracy);
    assert(gnss.have_vertical_accuracy);

    raw.fix_indicator = 0U;
    assert(Spresense::convert_gnss_sample(raw, gnss));
    assert(gnss.fix_status == Spresense::GnssFixStatus::NO_FIX);
    assert(gnss.latitude_e7 == 0);

    Spresense::ImuRawSample imu_raw {
        42U, 25.0f, 0.1f, -0.2f, 0.3f, 1.0f, -0.5f, 0.25f,
    };
    Spresense::ImuSample imu {};
    assert(Spresense::convert_imu_sample(imu_raw, imu));
    assert(near(imu.gyro_x_rad_s, 0.1f));
    assert(near(imu.accel_x_m_s2, 9.80665f));
    assert(near(imu.accel_y_m_s2, -4.903325f));

    imu_raw.accel_z_g = std::numeric_limits<float>::quiet_NaN();
    assert(!Spresense::convert_imu_sample(imu_raw, imu));
    assert(!Spresense::sensor_bridge_platform_ready());
    return 0;
}
