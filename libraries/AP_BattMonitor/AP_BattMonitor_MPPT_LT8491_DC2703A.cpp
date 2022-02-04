
#include "AP_BattMonitor_MPPT_LT8491_DC2703A.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Generic.h"
#include <utility>

// Constructor
AP_BattMonitor_MPPT_LT8491_DC2703A::AP_BattMonitor_LT8491_DC2703A(AP_BattMonitor &mon,
                                           AP_BattMonitor::BattMonitor_State &mon_state,
                                           AP_BattMonitor_Params &params,
                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
        : AP_BattMonitor_Backend(mon, mon_state, params),
        _dev(std::move(dev))
{
    _params._serial_number = AP_BATT_SERIAL_NUMBER_DEFAULT;
    _params._pack_capacity = 0;
}


void AP_BattMonitor_MPPT_LT8491_DC2703A::init(void)
{
    if (_dev) {
        timer_handle = _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_MPPT_LT8491_DC2703A::timer, void));
    }
}


void AP_BattMonitor_MPPT_LT8491_DC2703A::timer()
{

    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

    // read battery voltage (V)
    if (read_word(BATTMONITOR_LT8491_TELE_VBAT, data)) {
        _state.voltage = (float)data / 100.0f;
        MPPT_TELE.output.voltage = (float)data / 100.0f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // read current (A)
    if (read_word(BATTMONITOR_LT8491_TELE_IOUT, data)) {
        _state.current_amps = (float)data / 1000.0f;
        MPPT_TELE.output.current = (float)data / 1000.0f;
        _state.last_time_micros = tnow;
    }

    // update MPPT telemetry
    if (read_word(BATTMONITOR_LT8491_TELE_PIN, data)) {
        MPPT_TELE.input.power = (float)data / 100.0f;
    }


    if (read_word(BATTMONITOR_LT8491_TELE_POUT, data)) {
        MPPT_TELE.output.power = (float)data / 100.0f;
    }


    if (read_word(BATTMONITOR_LT8491_TELE_VIN, data)) {
        MPPT_TELE.input.voltage = (float)data / 100.0f;
    }


    if (read_word(BATTMONITOR_LT8491_TELE_IIN, data)) {
        MPPT_TELE.input.current = (float)data ;
    }

}


// read word from register
// returns true if read was successful, false if failed
bool AP_BattMonitor_MPPT_LT8491_DC2703A::read_word(uint8_t reg, uint16_t& data) const
{
    // buffer to hold results 
    const uint8_t read_size = 2;
    uint8_t buff[read_size];    // buffer to hold results

    // read the appropriate register from the device
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // convert buffer to word
    data = (uint16_t)buff[1]<<8 | (uint16_t)buff[0];

    // return success
    return true;
}