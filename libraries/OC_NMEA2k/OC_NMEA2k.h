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
        uint32_t id;         // Device ID
        Location location;
        float cog, cog_filt; // deg[0..360]
        float sog, sog_filt; // m/s
        float variation;        /*< Variation to true north (degrees)*/

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

        GPS() : id(0), cog(0), cog_filt(0), sog(0), sog_filt(0), variation(M_PI + 1), last_update(0),
                have_fix(false), hdop(0), vdop(0), num_sats(0), time_week(0), time_week_ms(0),
                time_last_update(0), 
                filter_cog(&(rover.g2.nmea2k.filt_cog)), filter_sog(&(rover.g2.nmea2k.filt_sog)) {}
    };

    class Triducer
    {
    public:
        uint32_t id;                            // Device ID
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

        Triducer() : id(0), water_depth(0), water_offset(0), water_range(0), water_temp(0),
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
        uint32_t id;         // Device ID
        float apparent_wind_angle;  // Apparent wind angle (deg[-180..180])
        float apparent_wind_angle_filt;
        float apparent_wind_speed;  // Apparent wind speed
        float apparent_wind_speed_filt;
        float water_wind_angle;      // (deg[-180..180])
        float water_wind_angle_filt;
        float water_wind_dir;      // (deg[0..360])
        float water_wind_dir_filt;
        float water_wind_speed;      // (m/s)
        float water_wind_speed_filt;
        float ground_wind_dir;        // TWD (deg[0..360])
        float ground_wind_dir_filt;   // (deg[0..360])
        float ground_wind_speed;      // TWS (m/s)
        float ground_wind_speed_filt; // (m/s)
        float wind_gusts;           // (m/s)

        float atmos_pressure;       // Atmospheric pressure (hPa)
        float air_temp;             // (celcius)
        float humidity;             // (%)
        uint64_t last_update;       // System time of last update (millis)

        FiltExpAng<float> filter_awa;   // Filter for apparent wind angle
        FiltExp<float> filter_aws;      // Filter for apparent wind speed
        FiltExpAng<float> filter_wwa;   // Filter for water wind angle
        FiltExpAng<float> filter_wwd;   // Filter for water wind direction
        FiltExp<float> filter_wws;      // Filter for water wind speed
        FiltExpAng<float> filter_gwd;   // Filter for ground wind direction
        FiltExp<float> filter_gws;      // Filter for ground wind speed

        WeatherStation() : id(0), apparent_wind_angle(0), apparent_wind_angle_filt(0),
                           apparent_wind_speed(0), apparent_wind_speed_filt(0),
                           water_wind_angle(0), water_wind_angle_filt(0),
                           water_wind_dir(0), water_wind_dir_filt(0),
                           water_wind_speed(0), water_wind_speed_filt(0),
                           ground_wind_dir(0), ground_wind_dir_filt(0),
                           ground_wind_speed(0), ground_wind_speed_filt(0),
                           wind_gusts(0),
                           atmos_pressure(0), air_temp(0), humidity(0),
                           last_update(0),
                           filter_aws(&(rover.g2.nmea2k.filt_aws)),
                           filter_awa(&(rover.g2.nmea2k.filt_awa),180),
                           filter_gws(&(rover.g2.nmea2k.filt_tws)),
                           filter_gwd(&(rover.g2.nmea2k.filt_twd)),
                           filter_wwa(&(rover.g2.nmea2k.filt_twa),180),
                           filter_wws(&(rover.g2.nmea2k.filt_twd)),
                           filter_wwd(&(rover.g2.nmea2k.filt_tws)) {}
    };

    class Compass
    {
    public:
        uint32_t id;         // Device ID
        float heading;          /*< True heading (degrees)*/
        float heading_filt;     /*< True heading (degrees)*/
        float magnetic;         /*< Magnetic heading (degrees)*/
        float variation;        /*< Variation to true north (degrees), Should not be used, use variation from a GPS instead */

        uint64_t last_update; // System time of last update (millis)

        float roll, pitch, yaw; // Attitude reported from the compass/Airmar (radians)

        FiltExpNlAng<float> filter_hdg;

        Compass() : id(0), heading(0), heading_filt(0),
                    magnetic(0), variation(M_PI+1), last_update(0),
                    roll(0), pitch(0), yaw(0),
                    filter_hdg(&(rover.g2.nmea2k.filt_hdg_tc),&(rover.g2.nmea2k.filt_hdg_nl)) {}
    };

    GPS primary_gps;   // Airmar
    GPS secondary_gps; // AIS Transceiver
    GPS tertiary_gps;  // Backup GPS
    Triducer triducer;
    WeatherStation weather;
    Compass primary_compass;
    Compass secondary_compass;

    void init();
    bool read();
    void timer();

private:
    AP_HAL::UARTDriver *_port;
    unsigned char msg[512];

    bool term_complete(unsigned int pgn, MsgVals *pmv);
    void update_status();

public:
    NMEA2K() : _port(nullptr) {}


    Location get_location();
    float get_heading();

    inline GPS* get_active_gps() {
        if (primary_gps.have_fix) {
            return &primary_gps;
        } else if (secondary_gps.have_fix) {
            return &secondary_gps;
        } else if (tertiary_gps.have_fix) {
            return &tertiary_gps;
        }
        return NULL;
    }

    inline Compass* get_active_compass() {
        uint64_t now = AP_HAL::millis64();
        if ((now - primary_compass.last_update) < 5000) { // Primary compass reading is < 5 seconds old
            return &primary_compass;
        } else if ((now - secondary_compass.last_update) < 5000) { // Secondary compass reading is < 5 seconds old
            return &secondary_compass;
        }
        return NULL;
    }

    inline float get_variation() {
        if (primary_gps.have_fix && primary_gps.variation < M_PI) {
            return primary_gps.variation;
        } else if (secondary_gps.have_fix && secondary_gps.variation < M_PI) {
            return secondary_gps.variation;
        } else if (tertiary_gps.have_fix && tertiary_gps.variation < M_PI) {
            return tertiary_gps.variation;
        }
        // TODO: find another source if all GPSs are down
        return 0;
    }
};

extern NMEA2K nmea2k_sensors;
