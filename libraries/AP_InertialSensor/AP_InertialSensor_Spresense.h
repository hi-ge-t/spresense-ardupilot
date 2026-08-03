#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_Spresense : public AP_InertialSensor_Backend {
public:
    explicit AP_InertialSensor_Spresense(AP_InertialSensor &imu);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu);

    void start() override;
    void accumulate() override;
    bool update() override;
    bool get_output_banner(char *banner, uint8_t banner_len) override;

private:
    // Match the already hardware-verified bounded Sony probe for M1.  A
    // flight-rate qualification is a later hardware gate.
    static constexpr uint16_t SAMPLE_RATE_HZ = 60U;
    static constexpr uint8_t MAX_DRAIN_SAMPLES = 8U;

    bool _started = false;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE
