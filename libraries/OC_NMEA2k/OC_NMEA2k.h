/*
 * OC_NMEA2k.h
 *
 *  Created on: 16 Dec. 2019
 *      Author: mmcgill
 */

#pragma once

#include <AP_Common/Location.h>
#include <AP_HAL/UARTDriver.h>

class MsgVals;
class NMEA2K {
public:
    class GPS {
    public:
        Location location;
        float cog;  // deg[0..360]
        float sog;  // m/s

        uint64_t last_update;           // System time of last update (millis)

        bool have_fix;                  ///< driver fix status
        uint16_t hdop;                  // horizontal precision (cm)
        uint16_t vdop;                  // vertical precision (cm)
        uint16_t num_sats;              // Number of satellites

        uint16_t time_week;                 ///< GPS week number
        uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
        uint64_t time_last_update;           // System time of last time update (millis)

        GPS() : cog(0), sog(0), last_update(0),
                have_fix(false), hdop(0), vdop(0), num_sats(0), time_week(0), time_week_ms(0),
                time_last_update(0) {}
    };

    class Triducer {
    public:
        float water_depth;      // m
        float water_temp;       // celcius
        float speed_thru_water; // m/s
        uint64_t last_update;

        Triducer() : water_depth(0), water_temp(0), speed_thru_water(0), last_update(0) {}
    };

    class WeatherStation {
    public:
        float wind_speed_true;       // m/s
        float wind_dir_true;         // deg[0..360]
        float air_pressure;
        float air_temp;         // celcius
        float humidity;
        uint64_t last_update;

        WeatherStation() : wind_speed_true(0), wind_dir_true(0), air_pressure(0), air_temp(0), humidity(0), last_update(0) {}
    };

    class Compass {
    public:
        float heading; /*< Magnetic heading (degrees)*/
        float variation; /*< Variation to true north (degrees)*/
        float deviation; /*< Deviation whatever that means.(degrees)*/
        float offset; /*< MAG_OFFSET parameter value (degrees)*/
        uint8_t reference; /*< Magnetic or true*/

        uint64_t last_update;

        Compass() : heading(0), variation(0), deviation(0), offset(0), reference(0), last_update(0) {}
    };

    GPS primary_gps;            // Airmar
    GPS secondary_gps;          // AIS Transceiver
    GPS tertiary_gps;           // Backup GPS
    Triducer triducer;
    WeatherStation weather;
    Compass compass;

    void init(AP_HAL::UARTDriver& port);
    bool read(AP_HAL::UARTDriver& port);

private:

    unsigned char msg[512];
//    NMEA2K() {}
    bool term_complete(unsigned int pgn, MsgVals *pmv);
};

extern NMEA2K nmea2k_sensors;
