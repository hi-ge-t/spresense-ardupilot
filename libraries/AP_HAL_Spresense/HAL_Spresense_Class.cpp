#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE

#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_Spresense_Class.h"
#include "RCOutput.h"
#include "Scheduler.h"
#include "SensorBridge.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"

namespace {

Spresense::UARTDriver serial0_driver("/dev/ttyS0");
Empty::UARTDriver serial1_driver;
Empty::UARTDriver serial2_driver;
Empty::UARTDriver serial3_driver;
Empty::UARTDriver serial4_driver;
Empty::UARTDriver serial5_driver;
Empty::UARTDriver serial6_driver;
Empty::UARTDriver serial7_driver;
Empty::UARTDriver serial8_driver;
Empty::UARTDriver serial9_driver;

Empty::I2CDeviceManager i2c_manager;
Empty::SPIDeviceManager spi_manager;
Empty::WSPIDeviceManager wspi_manager;
Empty::AnalogIn analog_in;
Spresense::Storage storage_driver;
Empty::GPIO gpio_driver;
Empty::RCInput rcin_driver;
Spresense::RCOutput rcout_driver;
Spresense::Scheduler scheduler_instance;
Spresense::Util util_instance;
Empty::OpticalFlow optical_flow_driver;
Empty::Flash flash_driver;

Spresense::HAL_Spresense hal_spresense;

} // namespace

Spresense::HAL_Spresense::HAL_Spresense() :
    AP_HAL::HAL(
        &serial0_driver,
        &serial1_driver,
        &serial2_driver,
        &serial3_driver,
        &serial4_driver,
        &serial5_driver,
        &serial6_driver,
        &serial7_driver,
        &serial8_driver,
        &serial9_driver,
        &i2c_manager,
        &spi_manager,
        &wspi_manager,
        &analog_in,
        &storage_driver,
        &serial0_driver,
        &gpio_driver,
        &rcin_driver,
        &rcout_driver,
        &scheduler_instance,
        &util_instance,
        &optical_flow_driver,
        &flash_driver,
        nullptr)
{
}

void Spresense::HAL_Spresense::run(
    int argc, char *const argv[], Callbacks *callbacks) const
{
    (void)argc;
    (void)argv;

    serial(0)->begin(115200U);
    if (!serial(0)->is_initialized()) {
        AP_HAL::panic("Spresense main UART startup failed");
    }
    if (!Spresense::sensor_bridge_platform_ready()) {
        AP_HAL::panic("Spresense sensor bridge unavailable");
    }
    console->printf("SPRESENSE_M1_COPTER_BOOT=HAL\n");
    console->flush();

    scheduler->init();
    if (!scheduler_instance.timing_healthy()) {
        console->printf("SPRESENSE_M1_COPTER_BOOT=SCHEDULER_FAIL\n");
        console->flush();
        AP_HAL::panic("Spresense scheduler startup failed");
    }
    storage->init();
    gpio->init();
    rcin->init();
    rcout->init();

    console->printf("SPRESENSE_M1_COPTER_BOOT=SETUP\n");
    console->flush();
    callbacks->setup();
    scheduler->set_system_initialized();
    console->printf("SPRESENSE_M1_COPTER_BOOT=LOOP\n");
    console->flush();

    uint8_t loop_trace_count = 0U;
    for (;;) {
        if (loop_trace_count == 0U) {
            console->printf("SPRESENSE_M1_COPTER_LOOP=CALL_1\n");
            console->flush();
        } else if (loop_trace_count == 1U) {
            console->printf("SPRESENSE_M1_COPTER_LOOP=CALL_2\n");
            console->flush();
        }
        callbacks->loop();
        if (loop_trace_count == 0U) {
            console->printf("SPRESENSE_M1_COPTER_LOOP=RETURN_1\n");
            console->flush();
        } else if (loop_trace_count == 1U) {
            console->printf("SPRESENSE_M1_COPTER_LOOP=RETURN_2\n");
            console->flush();
        }
        if (loop_trace_count < 2U) {
            loop_trace_count++;
        }
    }
}

const AP_HAL::HAL &AP_HAL::get_HAL()
{
    return hal_spresense;
}

AP_HAL::HAL &AP_HAL::get_HAL_mutable()
{
    return hal_spresense;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SPRESENSE
