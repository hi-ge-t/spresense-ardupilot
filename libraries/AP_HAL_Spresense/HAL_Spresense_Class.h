#pragma once

#include <AP_HAL/AP_HAL.h>

namespace Spresense {

class HAL_Spresense final : public AP_HAL::HAL {
public:
    HAL_Spresense();
    void run(int argc, char *const argv[], Callbacks *callbacks) const override;
};

} // namespace Spresense
