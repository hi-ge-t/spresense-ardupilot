#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_SPRESENSE_ENABLED

#include "GPS_Backend.h"

class AP_GPS_Spresense : public AP_GPS_Backend {
public:
    using AP_GPS_Backend::AP_GPS_Backend;

    bool read() override;
    bool get_lag(float &lag) const override;
    bool is_healthy() const override { return _healthy; }
    const char *name() const override { return "Spresense CXD5610"; }

private:
    bool _healthy = false;
};

#endif // AP_GPS_SPRESENSE_ENABLED
