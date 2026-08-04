#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE

#include "AP_InertialSensor_Spresense.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL_Spresense/SensorBridge.h>

#include <new>
#include <stdio.h>

AP_InertialSensor_Spresense::AP_InertialSensor_Spresense(
    AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_Spresense::probe(
    AP_InertialSensor &imu)
{
    if (!Spresense::pwbimu_start(SAMPLE_RATE_HZ)) {
        return nullptr;
    }
    return new (std::nothrow) AP_InertialSensor_Spresense(imu);
}

void AP_InertialSensor_Spresense::start()
{
    const uint32_t device_id = AP_HAL::Device::make_bus_id(
        AP_HAL::Device::BUS_TYPE_SPI, 5U, 0U,
        DEVTYPE_INS_CXD5602PWBIMU);
    if (!_imu.register_gyro(gyro_instance, SAMPLE_RATE_HZ, device_id) ||
        !_imu.register_accel(accel_instance, SAMPLE_RATE_HZ, device_id)) {
        return;
    }

    // The board-axis transform remains explicit and output-disabled until
    // its physical orientation is measured on the assembled airframe.
    set_gyro_orientation(gyro_instance, ROTATION_NONE);
    set_accel_orientation(accel_instance, ROTATION_NONE);
    _started = true;
}

void AP_InertialSensor_Spresense::accumulate()
{
    if (!_started) {
        return;
    }

    for (uint8_t count = 0U; count < MAX_DRAIN_SAMPLES; count++) {
        Spresense::ImuSample sample {};
        const auto result = Spresense::pwbimu_read(sample);
        if (result != Spresense::SensorReadStatus::SAMPLE) {
            break;
        }

        const uint64_t sample_us = AP_HAL::micros64();
        Vector3f accel(sample.accel_x_m_s2,
                       sample.accel_y_m_s2,
                       sample.accel_z_m_s2);
        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel, sample_us);
        _publish_temperature(accel_instance, sample.temperature_c);

        Vector3f gyro(sample.gyro_x_rad_s,
                      sample.gyro_y_rad_s,
                      sample.gyro_z_rad_s);
        _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);
    }
}

bool AP_InertialSensor_Spresense::update()
{
    if (!_started) {
        return false;
    }
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

bool AP_InertialSensor_Spresense::get_output_banner(char *banner,
                                                     uint8_t banner_len)
{
    snprintf(banner, banner_len, "IMU%u: Spresense CXD5602PWBIMU %uHz",
             gyro_instance, SAMPLE_RATE_HZ);
    return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE
