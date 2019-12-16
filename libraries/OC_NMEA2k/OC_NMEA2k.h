/*
 * OC_NMEA2k.h
 *
 *  Created on: 16 Dec. 2019
 *      Author: mmcgill
 */

#pragma once

#include <AP_Common/Location.h>
#include <AP_HAL/UARTDriver.h>

class NMEA2K {
public:
    class GPS {
    public:
        Location gps;
        float cog;  // deg[0..360]
        float sog;  // m/s
        uint64_t last_update;

        GPS() : cog(0), sog(0), last_update(0) {}
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
        float wind_speed;       // m/s
        float wind_dir;         // deg[0..360]
        float air_pressure;
        float air_temp;         // celcius
        float humidity;
        uint64_t last_update;

        WeatherStation() : wind_speed(0), wind_dir(0), air_pressure(0), air_temp(0), humidity(0), last_update(0) {}
    };

    class Compass {
    public:
        float heading; /*< Magnetic heading (degrees)*/
        float variation; /*< Variation to true north (degrees)*/
        float deviation; /*< Deviation whatever that means.(degrees)*/
        float offset; /*< MAG_OFFSET parameter value (degrees)*/
        uint8_t reference; /*< Magnetic or true*/

        uint64_t last_update;
    };

    GPS primary_gps, secondary_gps, tertiary_gps;
    Triducer triducer;
    WeatherStation weather;
    Compass compass;

    void read(AP_HAL::UARTDriver& port);
//    NMEA2K() {}
};

extern NMEA2K nmea2k_sensors;
