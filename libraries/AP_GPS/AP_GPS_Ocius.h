#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

/// NMEA parser
///
class AP_GPS_Ocius : public AP_GPS_Backend
{
public:
    AP_GPS_Ocius(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    virtual const char *name() const override {return "Ocius";}

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    bool        read() override;

    static bool _detect(struct NMEA_detect_state &state, uint8_t data);
};
