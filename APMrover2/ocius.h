/*
 * nmea2k.h
 *
 *  Created on: 30 Oct. 2019
 *      Author: mmcgill
 */

#pragma once

#include <AP_Common/Location.h>


#define KNOTS_PER_METRE     1.94384

#define CLIP_180(A)     (A < -180? A + 360: A > 180? A - 360: A)
#define CLIP_360(A)     (A < 0? A + 360: A > 360? A - 360: A)

enum SAIL_MODE {
    MOTOR_ONLY=0,
    MOTOR_SAIL=1,
    SAIL_ONLY=2,
    WAVE_POWER=3,
    MOTOR_SOLAR=5
};

#define SAIL_MIN_WIND   5   // Knots

enum WIND_STRENGTH {
    WIND_LOW = 1,
    WIND_FAIR = 2,
    WIND_STRONG = 4,
    WIND_HIGH = 8,

    WIND_GOOD = 6,
};

//enum MOTOR_HOMING_STATUS {
//    MOTOR_UNHOMED   = 0,
//    MOTOR_HOMED     = 1,
//    MOTOR_HOMING    = 2,
//};

enum SAIL_HOLD_MODE {
    HOLD_DRIFT      = 0,
    HOLD_ACTIVE     = 1,
    HOLD_FIGURE8    = 2,
};

enum VESSEL_TYPE {
    VESSEL_UNKNOWN      = 0,
    VESSEL_NEMO         = 1,
    VESSEL_STINGER      = 2,
};

enum SAIL_FLAGS {
    SAIL_NO_FLAGS = 0,
    SAIL_JIBE_ONLY = 1,
    SAIL_IGNORE_XTRACK = 2,
    SAIL_HEADING_TO_WP = 4,
};

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
//    NMEA2K() {}
};

//class EncodedServo {
//public:
//    uint16_t set_position;
//    uint16_t current_position;
//    uint32_t raw_position;
//    MOTOR_HOMING_STATUS homed;
//
//    EncodedServo() : set_position(0), current_position(0), raw_position(0), homed(MOTOR_UNHOMED) {}
//};
