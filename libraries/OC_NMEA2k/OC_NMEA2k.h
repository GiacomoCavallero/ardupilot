/*
 * OC_NMEA2k.h
 *
 *  Created on: 16 Dec. 2019
 *      Author: mmcgill
 */

#pragma once

#include <AP_Common/Location.h>
#include <AP_HAL/UARTDriver.h>
#include <OC_NMEA2k/oc_filter.h>


//#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
#include <../APMrover2/Rover.h>
//#endif

class MsgVals;
class NMEA2K
{
public:
    class GPS
    {
    public:
        Location location;
        float cog, cog_filt; // deg[0..360]
        float sog, sog_filt; // m/s

        uint64_t last_update; // System time of last update (millis)

        bool have_fix;     // fix status
        uint16_t hdop;     // horizontal precision (cm)
        uint16_t vdop;     // vertical precision (cm)
        uint16_t num_sats; // Number of satellites

        uint16_t time_week;        // GPS week number
        uint32_t time_week_ms;     // GPS time (milliseconds from start of GPS week)
        uint64_t time_last_update; // System time of last time update (millis)

        FiltExpAng<float> filter_cog;
        FiltExp<float> filter_sog;

        GPS() : cog(0), cog_filt(0), sog(0), sog_filt(0), last_update(0),
                have_fix(false), hdop(0), vdop(0), num_sats(0), time_week(0), time_week_ms(0),
                time_last_update(0), 
                filter_cog(&(rover.g2.nmea2k.filt_cog)), filter_sog(&(rover.g2.nmea2k.filt_sog)) {}
    };

    class Triducer
    {
    public:
        float water_depth;                      // (m)
        float water_offset;                     // (m) (+ve to water surface, -ve to bottom of keel)
        float water_range;                      // maximum measurable water depth (m)
        float water_temp;                       // (celcius)
        float longitudinal_speed_water;         // longitudinal speed (water reference) (m/s)
        float longitudinal_speed_water_filt;    // filtered longitudinal speed (water reference) (m/s)
        float transverse_speed_water;           // transverse speed (water reference) (m/s)
        float transverse_speed_water_filt;      // filtered transverse speed (water reference) (m/s)
        float longitudinal_speed_ground;        // longitudinal speed (ground reference) (m/s)
        float transverse_speed_ground;          // transverse speed (ground reference) (m/s)
        float stern_speed_water;                // stern speed (water reference) (m/s)
        float stern_speed_ground;               // stern speed (ground reference) (m/s)

        uint64_t last_update; // System time of last update (millis)

        FiltExp<float> filter_boatspeed;
        FiltExp<float> filter_leeway;

        Triducer() : water_depth(0), water_offset(0), water_range(0), water_temp(0),
                     longitudinal_speed_water(0), transverse_speed_water(0),
                     longitudinal_speed_water_filt(0), transverse_speed_water_filt(0),
                     longitudinal_speed_ground(0), transverse_speed_ground(0),
                     stern_speed_water(0), stern_speed_ground(0),
                     last_update(0), 
                     filter_boatspeed(&(rover.g2.nmea2k.filt_bsp)), 
                     filter_leeway(&(rover.g2.nmea2k.filt_lee)) {}
    };

    class WeatherStation
    {
    public:
        float wind_speed_true;      // (m/s)
        float wind_speed_true_filt; // (m/s)
        float wind_dir_true;        // (deg[0..360])
        float wind_dir_true_filt;   // (deg[0..360])
        float wind_gusts;           // (m/s)
        float atmos_pressure;       // Atmospheric pressure (hPa)
        float air_temp;             // (celcius)
        float humidity;             // (%)
        uint64_t last_update;       // System time of last update (millis)

        FiltExp<float> filter_tws;
        FiltExpAng<float> filter_twd;

        WeatherStation() : wind_speed_true(0), wind_speed_true_filt(0),
                           wind_dir_true(0), wind_dir_true_filt(0),
                           wind_gusts(0), atmos_pressure(0), air_temp(0), humidity(0),
                           last_update(0),
                           filter_tws(&(rover.g2.nmea2k.filt_tws)),
                           filter_twd(&(rover.g2.nmea2k.filt_twd)) {}
    };

    class Compass
    {
    public:
        float heading;          /*< True heading (degrees)*/
        float heading_filt;     /*< True heading (degrees)*/
        float magnetic;         /*< Magnetic heading (degrees)*/
        float variation;        /*< Variation to true north (degrees)*/

        uint64_t last_update; // System time of last update (millis)

        float roll, pitch, yaw; // Attitude reported from the compass/Airmar (radians)

        FiltExpNlAng<float> filter_hdg;

        Compass() : heading(0), heading_filt(0), 
                    magnetic(0), variation(0), last_update(0),
                    roll(0), pitch(0), yaw(0),
                    filter_hdg(&(rover.g2.nmea2k.filt_hdg_tc),&(rover.g2.nmea2k.filt_hdg_nl)) {}
    };

    GPS primary_gps;   // Airmar
    GPS secondary_gps; // AIS Transceiver
    GPS tertiary_gps;  // Backup GPS
    Triducer triducer;
    WeatherStation weather;
    Compass compass;

    void init();
    bool read();
    void timer();

private:
    AP_HAL::UARTDriver *_port;
    unsigned char msg[512];
    //    NMEA2K() {}
    bool term_complete(unsigned int pgn, MsgVals *pmv);
    void update_status();

public:
    NMEA2K() : _port(nullptr) {}
};

extern NMEA2K nmea2k_sensors;
