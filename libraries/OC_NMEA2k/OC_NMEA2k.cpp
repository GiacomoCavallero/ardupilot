/*
 * OC_NMEA2k.cpp
 *
 *  Created on: 16 Dec. 2019
 *      Author: mmcgill
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <OC_NMEA2k/OC_NMEA2k.h>
#include <stdio.h>
#include <map>
#include <algorithm>

#include "n2kparse.h"

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
#include <../APMrover2/Rover.h>
#endif

NMEA2K nmea2k_sensors;

#define NMEA2K_AIRMAR 36
#define NMEA2K_AIS 43
#define NMEA2K_SIMRAD 37  // To be set to ID of new water sensor
#define NMEA2K_BACKUP 127 // new gps/compass on foredeck ignore for now

#define NMEA2K_LATLONG_RESOLUTION 10000000

static uint32_t getBSDDate(const char *date)
{
    uint32_t year, month, day;
    sscanf(date, "%u.%u.%u", &year, &month, &day);
    uint32_t rval = day * 10000 + month * 100 + (year % 100);
    return rval;
}

static uint32_t getHMSInMillis(const char *time)
{
    int32_t hour, minute;
    float second;
    sscanf(time, "%u:%u:%f", &hour, &minute, &second);
    uint32_t rval = hour * 10000000 + minute * 100000 + second * 1000;
    return rval;
}

static void nmea2k_writeMessage(AP_HAL::UARTDriver &port, unsigned char command,
                                const unsigned char *cmd, const int len)
{
    unsigned char bst[255];
    unsigned char *b = bst;
    unsigned char *lenPtr;
    unsigned char crc;

    int i;

    *b++ = DLE;
    *b++ = STX;
    *b++ = command;
    crc = command;
    lenPtr = b++;

    for (i = 0; i < len; i++)
    {
        if (cmd[i] == DLE)
        {
            *b++ = DLE;
        }
        *b++ = cmd[i];
        crc += (unsigned char)cmd[i];
    }

    *lenPtr = i;
    crc += i;

    *b++ = (unsigned char)(256 - (int)crc);
    *b++ = DLE;
    *b++ = ETX;

    if ((int)port.write(bst, b - bst) != (int)(b - bst))
    {
    }
}

/**
   fill in time_week_ms and time_week from BCD date and time components
   assumes MTK19 millisecond form of bcd_time
 */
static void nmea2k_make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds,
                                 NMEA2K::GPS &state)
{
    uint8_t year, mon, day, hour, min, sec;
    uint16_t msec;

    year = bcd_date % 100;
    mon = (bcd_date / 100) % 100;
    day = bcd_date / 10000;

    uint32_t v = bcd_milliseconds;
    msec = v % 1000;
    v /= 1000;
    sec = v % 100;
    v /= 100;
    min = v % 100;
    v /= 100;
    hour = v % 100;

    int8_t rmon = mon - 2;
    if (0 >= rmon)
    {
        rmon += 12;
        year -= 1;
    }

    // get time in seconds since unix epoch
    uint32_t ret =
        (year / 4) - (GPS_LEAPSECONDS_MILLIS / 1000UL) + 367 * rmon / 12 + day;
    ret += year * 365 + 10501;
    ret = ret * 24 + hour;
    ret = ret * 60 + min;
    ret = ret * 60 + sec;

    // convert to time since GPS epoch
    ret -= 272764785UL;

    // get GPS week and time
    state.time_week = ret / AP_SEC_PER_WEEK;
    state.time_week_ms = (ret % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC;
    state.time_week_ms += msec;
}

#define GPS_ID                           \
    (pmv->src == gps_primary_id          \
         ? "Primary"                     \
         : (pmv->src == gps_secondary_id \
                ? "Secondary"            \
                : (pmv->src == gps_tertiary_id ? "Tertiary" : "Unknown")))

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool NMEA2K::term_complete(unsigned int pgn, MsgVals *pmv)
{
    // printf("Have complete term: %u\n", pgn);
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    uint32_t gps_primary_id = rover.g2.nmea2k.gps_1,
             gps_secondary_id = rover.g2.nmea2k.gps_2,
             gps_tertiary_id = rover.g2.nmea2k.gps_3;
#else
    uint32_t gps_primary_id = NMEA2K_AIRMAR, gps_secondary_id = NMEA2K_AIS,
             gps_tertiary_id = NMEA2K_BACKUP;
#endif

    GPS *gps_state = NULL;
    if (pmv->src == gps_primary_id)
    {
        gps_state = &primary_gps;
    }
    else if (pmv->src == gps_secondary_id)
    {
        gps_state = &secondary_gps;
    }
    else if (pmv->src == gps_tertiary_id)
    {
        gps_state = &tertiary_gps;
    }

    char method[64];
    char own_info[64];
    //    float windDir;

    //    if (pmv->src == NMEA2K_BACKUP) // FIXME: backup gps ignore for now
    //      return false;

    switch (pgn)
    {
    case 126992: // System Time
        if (gps_state != NULL && gps_state->have_fix)
        {
            nmea2k_make_gps_time(getBSDDate(pmv->getDateOrTime("Date")),
                                 getHMSInMillis(pmv->getDateOrTime("Time")),
                                 *gps_state);
            //                secondary_gps_state.time_week = gps_state->time_week;
            //                secondary_gps_state.time_week_ms =
            //                gps_state->time_week_ms;
            gps_state->time_last_update = AP_HAL::millis64();
        }
        break;

    case 127250: // Vessel Heading
        if (pmv->src == gps_primary_id)
        {
            // We only want the compass readings from the primary(Airmar) GPS.
            double heading = pmv->getDouble(
                "Heading"); // can be magnetic or true, depending on reference
            double declination = pmv->getDouble("Variation");
            int compass_reference = pmv->getInteger("Reference");
            compass.variation = ToDeg(declination);

            if (compass_reference)
            {
                // Magnetic
                compass.heading = CLIP_360(
                    ToDeg(heading + declination)); // assuming declincation is correct
                compass.magnetic = CLIP_360(ToDeg(heading));
            }
            else
            {
                // True
                compass.heading = CLIP_360(ToDeg(heading));
                compass.magnetic = CLIP_360(
                    ToDeg(heading - declination)); // assuming declincation is correct
            }
            compass.last_update = AP_HAL::millis64();
        }
        break;

    case 127257: // Attitude
        if (pmv->src == gps_primary_id)
        {
            // We only want the attitude readings from the primary(Airmar) GPS.
            compass.yaw = pmv->getDouble("Yaw");
            compass.pitch = pmv->getDouble("Pitch");
            compass.roll = pmv->getDouble("Roll");
        }
        break;

    case 129025:
        // Position Rapid Update
        if (gps_state != NULL)
        {
            if (pmv->isValid("Latitude") && pmv->isValid("Longitude"))
            {
                if (fabs(pmv->getDouble("Latitude")) > 0 &&
                    fabs(pmv->getDouble("Longitude")) > 0)
                {
                    gps_state->location.lat =
                        pmv->getDouble("Latitude") * NMEA2K_LATLONG_RESOLUTION;
                    gps_state->location.lng =
                        pmv->getDouble("Longitude") * NMEA2K_LATLONG_RESOLUTION;
                    if (!gps_state->have_fix)
                        gcs().send_text(MAV_SEVERITY_INFO,
                                        "Ocius N2K: GPS Fix(%s) restored from msg %d",
                                        GPS_ID, pgn);
                    gps_state->have_fix = true;
                    gps_state->last_update = AP_HAL::millis64();
                }
                else
                {
                    if (gps_state->have_fix)
                        gcs().send_text(MAV_SEVERITY_WARNING,
                                        "Ocius N2K: No GPS FIX(%s) from msg %d", GPS_ID,
                                        pgn);
                    gps_state->have_fix = false;
                }
            }
            else
            {
                if (gps_state->have_fix)
                    gcs().send_text(MAV_SEVERITY_WARNING,
                                    "Ocius N2K: No GPS FIX(%s) from msg %d", GPS_ID, pgn);
                gps_state->have_fix = false;
            }
        }
        break;

    case 129026:
        // COG SOG Rapid Update
        if (gps_state != NULL) // AIRMAR
        {
            if (pmv->isValid("SOG") && pmv->isValid("COG"))
            {
                if (pmv->getInteger("COG Reference") ==
                    0) // 0 True north, 2 Mag north, 3 is bad data
                {
                    // state.ground_speed     = pmv->getDouble("SOG");
                    // state.ground_course = ToDeg(pmv->getDouble("COG"));
                    double gs = pmv->getDouble("SOG");
                    double gc = ToDeg(pmv->getDouble("COG"));
                    if (gs > 655.34)
                    {
                        // INVALID ground speed
                    }
                    else
                    {
                        gps_state->cog = static_cast<float>(gc);
                        gps_state->sog = static_cast<float>(gs);

                        // TODO: Do we want to average the cog/sog again?

                        //                        gps_state->average.push_reading(gc, gs);
                        //                        gps_state->cog  =
                        //                        gps_state->average.speed; gps_state->sog
                        //                        = gps_state->average.direction;

                        gps_state->last_update = AP_HAL::millis64();
                    }
                }
            }
        }
        break;

    case 129029:
        // GNSS Position Data
        strcpy(method, pmv->getLookup("Method"));
        if (gps_state != NULL)
        {
            if (!strcmp(method, "no GNSS"))
            {
                if (gps_state->have_fix)
                    gcs().send_text(MAV_SEVERITY_WARNING,
                                    "Ocius N2K: No GNSS FIX(%s) from msg %d", GPS_ID,
                                    pgn);
                gps_state->have_fix = false;
            }
            else
            {
                if (!gps_state->have_fix)
                    gcs().send_text(MAV_SEVERITY_INFO,
                                    "Ocius N2K: GNSS Fix(%s) restored from msg %d",
                                    GPS_ID, pgn);
                gps_state->have_fix = true;
                gps_state->location.lat =
                    pmv->getDouble("Latitude") * NMEA2K_LATLONG_RESOLUTION;
                gps_state->location.lng =
                    pmv->getDouble("Longitude") * NMEA2K_LATLONG_RESOLUTION;
                gps_state->location.alt = pmv->getDouble("Altitude") * 100;
                gps_state->hdop = pmv->getDouble("HDOP") * 100;
                nmea2k_make_gps_time(getBSDDate(pmv->getDateOrTime("Date")),
                                     getHMSInMillis(pmv->getDateOrTime("Time")),
                                     *gps_state);
                gps_state->last_update = gps_state->time_last_update =
                    AP_HAL::millis64();
                gps_state->num_sats = pmv->getInteger("Number of SVs");
            }
        }
        break;

    case 129033: // Time and Date
        if (gps_state != NULL && gps_state->have_fix)
        {
            nmea2k_make_gps_time(getBSDDate(pmv->getDateOrTime("Date")),
                                 getHMSInMillis(pmv->getDateOrTime("Time")),
                                 *gps_state);
            gps_state->time_last_update = AP_HAL::millis64();
        }
        break;

    case 129039:
        // AIS Class B Position Report - includes self report from AIS
        if (gps_state != NULL)
        {
            strcpy(own_info, pmv->getLookup("AIS Transceiver information"));
            if (!strncmp(own_info, "Own information", 15))
            {
                if (pmv->isValid("Latitude") && pmv->isValid("Longitude"))
                {
                    //                    ais_gps_state.last_gps_time_ms =
                    //                    AP_HAL::millis();
                    gps_state->location.lat =
                        pmv->getDouble("Latitude") * NMEA2K_LATLONG_RESOLUTION;
                    gps_state->location.lng =
                        pmv->getDouble("Longitude") * NMEA2K_LATLONG_RESOLUTION;
                    gps_state->location.alt = 0;
                    if (!gps_state->have_fix)
                        gcs().send_text(MAV_SEVERITY_INFO,
                                        "Ocius N2K: AIS GPS Fix(%s) restored from msg %d",
                                        GPS_ID, pgn);
                    gps_state->have_fix = true;
                    gps_state->last_update = AP_HAL::millis64();
                }
                else
                {
                    if (gps_state->have_fix)
                        gcs().send_text(MAV_SEVERITY_WARNING,
                                        "Ocius N2K: No AIS GPS FIX(%s) from msg %d", GPS_ID,
                                        pgn);
                    gps_state->have_fix = false;
                }
                if (pmv->isValid("SOG") && pmv->isValid("COG"))
                {
                    double gs = pmv->getDouble("SOG");
                    double gc = ToDeg(pmv->getDouble("COG"));
                    gps_state->cog = static_cast<float>(gc);
                    gps_state->sog = static_cast<float>(gs);

                    // TODO: Do we want to average the cog/sog again?

                    //                        gps_state->average.push_reading(gc, gs);
                    //                        gps_state->cog  =
                    //                        gps_state->average.speed; gps_state->sog =
                    //                        gps_state->average.direction;
                }
            }
        }
        break;

    case 129539:
        // GNSS DOPs
        if (gps_state != NULL)
        {
            if (gps_state->have_fix)
            {
                gps_state->hdop = pmv->getDouble("HDOP") * 100;
                gps_state->vdop = pmv->getDouble("VDOP") * 100;
            }
        }
        break;

    case 129540:
        // GNSS Sats In View
        //          airmar_gps_state.num_sats      = pmv->getInteger("Sats in
        //          View"); gcs().send_text(MAV_SEVERITY_WARNING, "Airmar N2K:
        //          Sats in view %d", pmv->getInteger("Sats in View"));
        //            gpsUpdated = true;
        break;

    case 130306: // Wind Data
        if (pmv->src == gps_primary_id &&
            primary_gps.have_fix) // Ignore wind data when AIRMAR has no GPS fix
        {
            double wind_speed = pmv->getDouble("Wind Speed");
            double wind_angle = ToDeg(pmv->getDouble("Wind Angle"));
            if (wind_speed > 655.34 || wind_speed < 0 || wind_angle < -180 ||
                wind_angle > 360)
            {
                // INVALID readings ignore
            }
            else
            {
                switch (pmv->getInteger("Reference"))
                {
                case 0: // True wind
                    //weather.wind_dir_true = wrap_360(wind_angle);
                    //weather.wind_speed_true = wind_speed;

                    // Filter TWS and TWD
                    weather.wind_dir_true = weather.filt_wind_dir.filterPoint(wind_angle);
                    weather.wind_speed_true = weather.filt_wind_spd.filterPoint(wind_speed);


                    // TODO: Do we wish to average the wind speed/direction?
                    //weather.wind_average.push_reading(weather.wind_dir_true,
                    //                                  weather.wind_speed_true);
                    break;
                case 1: // Magnetic
                case 2: // Apparent
                case 3: // True (boat referenced)
                case 4: // True (water referenced)
                        // TODO: consider use of other readings
                default:
                    break;
                }
            }
        }
        else
        {
            // if we have no fix then ignore wind readings
        }

        break;

    case 128259: // Speed
        if (fabs(triducer.transverse_speed_water) > FLT_EPSILON)
            triducer.longitudinal_speed_water =
                pmv->getDouble("Speed Water Referenced");
        break;
    case 130578: // Vessel Speed Components
        triducer.longitudinal_speed_water =
            pmv->getDouble("Longitudinal Speed, Water-referenced");
        triducer.transverse_speed_water =
            pmv->getDouble("Transverse Speed, Water-referenced");
        triducer.longitudinal_speed_ground =
            pmv->getDouble("Longitudinal Speed, Ground-referenced");
        triducer.transverse_speed_ground =
            pmv->getDouble("Transverse Speed, Ground-referenced");
        triducer.stern_speed_water =
            pmv->getDouble("Stern Speed, Water-referenced");
        triducer.stern_speed_ground =
            pmv->getDouble("Stern Speed, Ground-referenced");

        // Filter boatspeed and leeway
        triducer.longitudinal_speed_water = triducer.filt_bsp_longitudinal.filterPoint(triducer.longitudinal_speed_water);
        triducer.transverse_speed_water = triducer.filt_bsp_transverse.filterPoint(triducer.transverse_speed_water);
        break;

    case 128267: // Water Depth
        triducer.water_depth = pmv->getDouble("Depth");
        triducer.water_offset = pmv->getDouble("Offset");
        triducer.water_range = pmv->getDouble("Range");
        break;

    case 130311:                                        // Environmental Parameters
        if (pmv->getInteger("Temperature Source") == 0) // Water temp
        {
            triducer.water_temp = pmv->getDouble("Temperature");
        }
        else if (pmv->getInteger("Temperature Source") == 1) // Outside temp
        {
            weather.air_temp = pmv->getDouble("Temperature");
        }
        if (pmv->getInteger("Humidity Source") == 1) // Outside humidity
        {
            weather.humidity = pmv->getDouble("Humidity");
            weather.atmos_pressure = pmv->getDouble("Atmospheric Pressure");
        }
        break;
    case 130312: // Temperature
                 //    if (pmv->getInteger("Temperature Source") == 0)
                 //    {
                 //        triducer.water_temp = pmv->getDouble("Actual
                 //        Temperature");
                 //    }
        break;
    case 130323: // Meteorological Station Data
    {
        double wind_speed = pmv->getDouble("Wind Speed");
        double wind_angle = ToDeg(pmv->getDouble("Wind Direction"));
        double wind_gusts = pmv->getDouble("Wind Gusts");
        if (pmv->getInteger("Wind Reference") == 0)
        { // True wind
            //weather.wind_dir_true = wrap_360(wind_angle);
            //weather.wind_speed_true = wind_speed;
            weather.wind_gusts = wind_gusts;

            weather.wind_dir_true   = weather.filt_wind_dir.filterPoint(wind_angle);
            weather.wind_speed_true = weather.filt_wind_spd.filterPoint(wind_speed);
        }
        double ambient_temp = pmv->getDouble("Ambient Temperature");
        double atmos_pressure = pmv->getDouble("Atmospheric Pressure");
        weather.air_temp = ambient_temp;
        weather.atmos_pressure = atmos_pressure;
    }
    break;

    case 59904:  // ISO Request
    case 60928:  // ISO Address Claim
    case 65410:  // Airmar: Device Information
    case 127251: // Rate of Turn
    case 127258: // Magnetic Variation
    case 129044: // Datum
    // case 130311: // Environmental Parameters
    case 130313: // Humidity  // TODO: Does it differ from in 130311?
    case 130314: // Actual Pressure // TODO: Does it differ from in 130311?
                 //        case 130323: // Meteorological Station Data // TODO:
                 //        Does it differ from in 130311?
    case 130944: // Airmar: POST
    case 0:      // Ignore
    case 130945: // Fast Packet Transfer
                 // TODO: Look up these PGNs
    case 65408:  // Airmar: Depth Quality Factor
    case 65409:  // Airmar: Speed Pulse Count
    case 128275: // Distance Log
        break;
    case 129038: // AIS Class A Position Report
    case 129041: // AIS Aids to Navigation (AtoN) Report
    case 129794: // AIS Class A Static and Voyage Related Data

    case 129809: // AIS Class B static data (msg 24 Part A)
    case 129810: // AIS Class B static data (msg 24 Part B)
    case 130934: // ???
    case 130935: // ???
        break;

    default:
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Ocius N2K received pgn %d(0x%x) from source %u.", pgn, pgn,
                        pmv->src);
        break;
    }

    update_status();

    return false;
}

void NMEA2K::update_status()
{
    // Check GPS Status here
    uint64_t tnow = AP_HAL::millis64();
    uint64_t tdiff = tnow - primary_gps.last_update;
    if (tdiff > 5000)
    {
        // we have not had an airmar message for 5 seconds, it may be powered off
        // they should come every 2 seconds from the GME ais unit
        if (primary_gps.have_fix)
        {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Ocius N2K: GPS(Primary) TIMEOUT, no GPS update in 5s");
            primary_gps.have_fix = false;
        }
    }
    tdiff = tnow - secondary_gps.last_update;
    if (tdiff > 5000)
    {
        // we have not had a secondary GPS message for 5 seconds, it may be powered
        // off they should come every 2 seconds from the GME ais unit
        if (secondary_gps.have_fix)
        {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Ocius N2K: GPS(Secondary) TIMEOUT, no GPS update in 5s");
            secondary_gps.have_fix = false;
        }
    }
    tdiff = tnow - tertiary_gps.last_update;
    if (tdiff > 5000)
    {
        // we have not had an tertiary GPS message for 5 seconds, it may be powered
        // off they should come every 2 seconds from the GME ais unit
        if (tertiary_gps.have_fix)
        {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Ocius N2K: GPS(Tertiary) TIMEOUT, no GPS update in 5s");
            tertiary_gps.have_fix = false;
        }
    }
}

void NMEA2K::init()
{
    if (_port == nullptr)
    {
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
        _port = rover.serial_manager.find_serial(
            AP_SerialManager::SerialProtocol_NMEA2K, 0);
#endif
        if (_port == nullptr)
        {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Ocius N2K: No NMEA2K serial device located.");
            return;
        }
        nmea2k_writeMessage(*_port, NGT_MSG_SEND, NGT_STARTUP_SEQ,
                            sizeof(NGT_STARTUP_SEQ));
        hal.scheduler->register_io_process(
            FUNCTOR_BIND(this, &NMEA2K::timer, void));
    }
}

void NMEA2K::timer() { read(); }

bool NMEA2K::read()
{
    if (_port == nullptr)
    {
        update_status();
        return false;
    }
    bool parsed = false;
    int16_t numc = _port->available();
    if (numc == 0)
    {
        update_status();
        return false;
    }

    while (numc)
    {
        unsigned char c = _port->read();
        int len = readNGT1Byte(c, msg);
        if (len)
        {
            // we have a complete NGT message
            int command = 0;
            int payLen = 0;
            int mtype = messageReceived(msg, len, command, payLen);
            if (mtype == 1)
            {
                // n2k message
                MsgVals *pmv = 0;
                unsigned int pgn = n2kMessageReceived(msg + 2, payLen, pmv);
                if (pmv)
                {
                    term_complete(pgn, pmv);
                    parsed = true;
                    delete pmv;
                }
            }
        }
        numc--;
    }
    return parsed;
}

template <typename T>
FiltVar<T>::FiltVar(T var)
{
    // Construct a timestamped var
    var_ = var;
    timestamp_ = std::chrono::system_clock::now();
}

template <typename T>
FiltVar<T>::FiltVar()
{
    // Construct a timestamp, Leave var_ default initialized
    timestamp_ = std::chrono::system_clock::now();
}

template <typename T>
T FiltBoxcar<T>::filterPoint(T point)
{
    // Add new point to deque
    sample.push_front(FiltVar<T>(point));

    // Trim off old points
    while (!sample.empty() &&
           std::chrono::duration<double>(point.timestamp() - sample.back().timestamp()).count() > *bctime)
    {
        sample.pop_back();
    }

    // Return boxcar averaged value
    T acc;
    for (auto i : sample)
    {
        acc += i.var();
    }
    size_t n = sample.size();

    return n ? acc / n : 0;
}

template <typename T>
T FiltBoxcarAng<T>::filterPoint(T angle)
{
    // Convert directional value to u/v, filter, convert back and wrap to appropiate range
    Vector2<T> dataPoint(dToUv(angle));
    Vector2<T> filtVec = FiltBoxcar<T>::filterPoint(dataPoint);

    T angleFiltered = ToDeg(filtVec.angle());
    return mod_ == 180 ? wrap_180(angleFiltered) : wrap_360(angleFiltered);
}

template <typename T>
T FiltExp<T>::filterPoint(T point)
{
    // Create new datapoint and calculate smoothing constant based on time difference
    FiltVar<T> newPoint(point);
    double dTime = std::chrono::duration<double>(newPoint.timestamp() - oldPoint.timestamp()).count();
    double a = 0;
    if (fpclassify(*tau) != FP_ZERO)
        a = std::exp(-dTime / (*tau));

    // Return filtered value and store for next time
    T newVal = oldPoint.var() * a + newPoint.var() * (1 - a);
    newPoint.setVar(newVal);
    oldPoint = newPoint;
    return newVal;
}

template <typename T>
T FiltExpAng<T>::filterPoint(T angle)
{
    // Convert directional value to u/v, filter, convert back and wrap to appropiate range
    Vector2<T> dataPoint(dToUv(angle));
    Vector2<T> filtVec = this->FiltExp<Vector2<T>>::filterPoint(dataPoint);

    T angleFiltered = ToDeg(filtVec.angle());
    return mod_ == 180 ? wrap_180(angleFiltered) : wrap_360(angleFiltered);
}

template <typename T>
T FiltExpNl<T>::filterPoint(T point)
{
    // Create new datapoint and calculate smoothing constant based on time difference 
    // If new point is outside bound from oldPoint, reduce smoothing constant

    FiltVar<T> newPoint(point);
    double dTime = std::chrono::duration<double>(newPoint.timestamp() - oldPoint.timestamp()).count();

    double a = 0;
    if (fpclassify(*tau) != FP_ZERO)
        a = std::exp(-dTime / *(tau));

    double bFac = 0;
    if (bound)
        bFac = abs((newPoint.val() - oldPoint.val()) / bound);

    if (bFac > 1)
    {
        // Reduce damping to 0 by the time we are 8 times bounds - for faster
        // response during manouvers
        a = std::max<double>(0, (-a / 7) * (bFac - 1) + a);
    }

    // Return exponential filtered value and store for next time
    T newVal = a * oldPoint.var() + (1 - a) * newPoint.var();
    newPoint.setVar(newVal);
    oldPoint = newPoint;
    return newVal;
}

template <typename T>
T FiltExpNlAng<T>::filterPoint(T angle)
{
    // Convert directional value to u/v, filter, convert back and wrap to appropiate range
    Vector2<T> dataPoint(dToUv(angle));
    Vector2<T> filtVec = FiltExpNl<T>::filterPoint(dataPoint);

    T angleFiltered = ToDeg(filtVec.angle());
    return mod_ == 180 ? wrap_180(angleFiltered) : wrap_360(angleFiltered);
}

double calCompass(double h)
{
    // Setup deviation table
    // Eventually from file
    std::map<int,double> devTable;
    devTable.insert({  0, 1});
    devTable.insert({ 30,-1});
    devTable.insert({ 60, 2});
    devTable.insert({ 90, 4});
    devTable.insert({120, 0});
    devTable.insert({180,-3});
    devTable.insert({240,-1});
    devTable.insert({280, 1});
    devTable.insert({330, 3});

    // Lookup table and interpolate
    int h1, h2;
    double c1, c2;

    auto it = devTable.lower_bound(h);

    if (it == devTable.begin()) {
        h1 = std::prev(devTable.end())->first - 360;
        c1 = std::prev(devTable.end())->second;
        h2 = it->first;
        c2 = it->second;
    }
    else if (it == devTable.end()) {
        h1 = std::prev(devTable.end())->first;
        c1 = std::prev(devTable.end())->second;
        h2 = devTable.begin()->first + 360;
        c2 = devTable.begin()->second;
    }
    else {
        h1 = std::prev(it)->first;
        c1 = std::prev(it)->second;
        h2 = it->first;
        c2 = it->second;
    }

    double c = linInterp(h1,h2,c1,c2,h);
    return wrap_360(h + c);
}

double linInterp(double h1, double h2, double c1, double c2, double h)
{
    double c = c1;

    if (fpclassify(h2 - h1) != FP_ZERO) {
        c =  c1 + (c2-c1) * (h-h1) / (h2-h1);
    }
    return c;
    
}