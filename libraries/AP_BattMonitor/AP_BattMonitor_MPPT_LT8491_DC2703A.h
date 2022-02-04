#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <utility>

#define AP_BATTMONITOR_LT8491_I2C_ADDR               0x10 // Set by pull ups loaded on DC2703A board

class AP_BattMonitor_MPPT_LT8491_DC2703A : public AP_BattMonitor_Backend
{
public:

    // I2C Registers
    enum BATTMONITOR_LT8491 {
       BATTMONITOR_LT8491_TELE_TBAT = 0x00,     // Temperature 
       BATTMONITOR_LT8491_TELE_POUT = 0x02,     // Output Power
       BATTMONITOR_LT8491_TELE_PIN  = 0x04,     // Input Power
       BATTMONITOR_LT8491_TELE_EFF  = 0x06,     // Charger efficiency
       BATTMONITOR_LT8491_TELE_IOUT = 0x08,     // Output current
       BATTMONITOR_LT8491_TELE_IIN  = 0x0A,     // Input current
       BATTMONITOR_LT8491_TELE_VBAT = 0x0C,     // Battery voltage
       BATTMONITOR_LT8491_TELE_VIN  = 0x0E,     // Input voltage measured from the FBIN pin
       BATTMONITOR_LT8491_TELE_VINR = 0x10,     // Input voltage measured from VINR pin 
    };

    struct MPPT_TELE{
        float voltage;
        float current;
        float power;
    } input, output;
    /// Constructor
    AP_BattMonitor_MPPT_LT8491_DC2703A(AP_BattMonitor &mon,
                    AP_BattMonitor::BattMonitor_State &mon_state,
                    AP_BattMonitor_Params &params,
                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // virtual destructor to reduce compiler warnings
    virtual ~AP_BattMonitor_MPPT_LT8491_DC2703A() {}

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    bool has_temperature() const override { return _has_temperature; }

    // all smart batteries are expected to provide current
    bool has_current() const override { return true; }

    // don't allow reset of remaining capacity for SMBus
    bool reset_remaining(float percentage) override { return false; }

    // return true if cycle count can be provided and fills in cycles argument
    bool get_cycle_count(uint16_t &cycles) const override;

    virtual void init(void) override;

protected:

    void read(void) override;

    // reads the pack full charge capacity
    // returns true if the read was successful, or if we already knew the pack capacity
    bool read_full_charge_capacity(void);

    // reads the remaining capacity
    // returns true if the read was successful, which is only considered to be the
    // we know the full charge capacity
    bool read_remaining_capacity(void);

    // return a scaler that should be multiplied by the battery's reported capacity numbers to arrive at the actual capacity in mAh
    virtual uint16_t get_capacity_scaler() const { return 1; }

    // reads the temperature word from the battery
    // returns true if the read was successful
    virtual bool read_temp(void);

    // reads the serial number if it's not already known
    // returns true if the read was successful, or the number was already known
    bool read_serial_number(void);

    // reads the battery's cycle count
    void read_cycle_count();

     // read word from register
     // returns true if read was successful, false if failed
    bool read_word(uint8_t reg, uint16_t& data) const;

    // get_PEC - calculate PEC for a read or write from the battery
    // buff is the data that was read or will be written
    uint8_t get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    bool _pec_supported; // true if PEC is supported

    int32_t _serial_number = -1;    // battery serial number
    uint16_t _full_charge_capacity; // full charge capacity, used to stash the value before setting the parameter
    bool _has_cell_voltages;        // smbus backends flag this as true once they have received a valid cell voltage report
    uint16_t _cycle_count = 0;      // number of cycles the battery has experienced. An amount of discharge approximately equal to the value of DesignCapacity.
    bool _has_cycle_count;          // true if cycle count has been retrieved from the battery
    bool _has_temperature;

    virtual void timer(void) = 0;   // timer function to read from the battery

    AP_HAL::Device::PeriodicHandle timer_handle;
};