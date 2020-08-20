/*
 * OC_NMEA2k.h
 *
 *  Created on: 16 Dec. 2019
 *      Author: mmcgill
 */

#pragma once

#include <AP_Common/Location.h>
#include <AP_HAL/UARTDriver.h>
#include <deque>
#include <chrono>

struct vector_average_t
{
    //static const uint32_t MAX_READINGS = 5*5;
    uint32_t MAX_READINGS;
    float direction; // degrees
    float speed;     // mps

    // The latest measurements
    //float directions[MAX_READINGS];
    //float speeds[MAX_READINGS];
    float *directions;
    float *speeds;
    uint32_t idx;

    vector_average_t(uint32_t mr);

    ~vector_average_t()
    {
        delete[] directions;
        delete[] speeds;
        delete[] sind;
        delete[] cosd;
    }

    // vectors for the measurements
    //float sind[MAX_READINGS];
    //float cosd[MAX_READINGS];
    float *sind;
    float *cosd;

    void push_reading(float dir_deg, float spd_mps);
};

class MsgVals;
class NMEA2K
{
public:
    class GPS
    {
    public:
        Location location;
        float cog; // deg[0..360]
        float sog; // m/s

        uint64_t last_update; // System time of last update (millis)

        bool have_fix;     ///< driver fix status
        uint16_t hdop;     // horizontal precision (cm)
        uint16_t vdop;     // vertical precision (cm)
        uint16_t num_sats; // Number of satellites

        uint16_t time_week;        ///< GPS week number
        uint32_t time_week_ms;     ///< GPS time (milliseconds from start of GPS week)
        uint64_t time_last_update; // System time of last time update (millis)

        GPS() : cog(0), sog(0), last_update(0),
                have_fix(false), hdop(0), vdop(0), num_sats(0), time_week(0), time_week_ms(0),
                time_last_update(0) {}
    };

    class Triducer
    {
    public:
        float water_depth;               // (m)
        float water_offset;              // (m) (+ve to water surface, -ve to bottom of keel)
        float water_range;               // maximum measurable water depth (m)
        float water_temp;                // (celcius)
        float longitudinal_speed_water;  // longitudinal speed (water reference) (m/s)
        float transverse_speed_water;    // transverse speed (water reference) (m/s)
        float longitudinal_speed_ground; // longitudinal speed (ground reference) (m/s)
        float transverse_speed_ground;   // transverse speed (ground reference) (m/s)
        float stern_speed_water;         // stern speed (water reference) (m/s)
        float stern_speed_ground;        // stern speed (ground reference) (m/s)

        uint64_t last_update; // System time of last update (millis)

        Triducer() : water_depth(0), water_offset(0), water_range(0), water_temp(0),
                     longitudinal_speed_water(0), transverse_speed_water(0),
                     longitudinal_speed_ground(0), transverse_speed_ground(0),
                     stern_speed_water(0), stern_speed_ground(0),
                     last_update(0) {}
    };

    class WeatherStation
    {
    public:
        float wind_speed_true; // (m/s)
        float wind_dir_true;   // (deg[0..360])
        float wind_gusts;      // (m/s)
        float atmos_pressure;  // Atmospheric pressure (hPa)
        float air_temp;        // (celcius)
        float humidity;        // (%)
        uint64_t last_update;  // System time of last update (millis)

        vector_average_t wind_average;

        WeatherStation() : wind_speed_true(0), wind_dir_true(0), wind_gusts(0),
                           atmos_pressure(0), air_temp(0), humidity(0),
                           last_update(0),
                           wind_average(25) {}
    };

    class Compass
    {
    public:
        float heading;   /*< True heading (degrees)*/
        float magnetic;  /*< Magnetic heading (degrees)*/
        float variation; /*< Variation to true north (degrees)*/

        uint64_t last_update; // System time of last update (millis)

        float roll, pitch, yaw; // Attitude reported from the compass/Airmar (radians)

        Compass() : heading(0), magnetic(0), variation(0), last_update(0),
                    roll(0), pitch(0), yaw(0) {}
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

template <typename T>
class FiltVar
{
    // Class for variables to be filtered. Adds timestamp to data
private:
    std::time_t timestamp_;
    T var_;

public:
    FiltVar(T var);
    void setVar(T var) { var_ = var; }
    std::time_t timestamp() { return timestamp_; }
    T var() { return var_; }
};

template <typename T>
class FiltBoxcar
{
    // Boxcar filtering class for scalar values
public:
    FiltBoxcar(std::chrono::seconds secs) : bctime{secs} {} // Creates empty deque and sets up parameters
    T addPoint(T point);                           // Adds variable, trims old and returns filtered value

private:
    std::chrono::seconds bctime;
    std::deque<FiltVar<T>> sample;
};

template <typename T>
class FiltBoxcarAng : FiltBoxcar<T>
{
    // Boxcar filtering class for angular data
    // mod = 360: returns angle in 0 to 360 range
    // mod = 180: returns angle in -180 to 180 range
public:
    FiltBoxcarAng(time_t secs, int mod = 360) : FiltBoxcar<T>::bctime{secs}, mod_{mod} {} // Creates empty deque and sets up parameters
    T addPoint(T angle);

private:
    int mod_;
};

template <typename T>
Vector2<T> dToUv(T d)
{
    return Vector2<T>(sin(radians(d)), cos(radians(d)));
}

template <typename T>
class FiltExp
{
    // Exponential filtering class for scalar values
public:
    FiltExp(double secs) : tau{secs} { } // Creates initial lastPoint and sets up parameters
    T addPoint(T point);                        // Adds new data point and returns filtered value

private:
    double tau;
    FiltVar<T> oldPoint;
};

template <typename T>
class FiltExpAng : FiltExp<T>
{
    // Exponential filtering class for angular data
    // mod = 360: returns angle in 0 to 360 range
    // mod = 180: returns angle in -180 to 180 range
public:
    FiltExpAng(double secs, int mod = 360) : FiltExp<T>::tau{secs}, mod_{mod} {} // Creates initlal lastPoint and sets up parameters
    T addPoint(T angle);

private:
    int mod_;
};

template <typename T>
class FiltExpNl
{
    // Non-linear exponential filtering class for scalar values
public:
    FiltExpNl(double secs, T bound_) : tau{secs}, bound{bound_} { } // Creates initial lastPoint and sets up parameters
    T addPoint(T point);                        // Adds new data point and returns filtered value

private:
    double tau;
    T bound;
    FiltVar<T> oldPoint;
};

template <typename T>
class FiltExpNlAng : FiltExpNl<T>
{
    // Non-linear exponential filtering class for angular data
    // mod = 360: returns angle in 0 to 360 range
    // mod = 180: returns angle in -180 to 180 range
public:
    FiltExpNlAng(double secs, T bound_, int mod = 360) : FiltExpNl<T>::tau{secs}, FiltExpNl<T>::bound{bound_}, mod_{mod} {} // Creates initial lastPoint and sets up parameters
    T addPoint(T angle);

private:
    int mod_;
};