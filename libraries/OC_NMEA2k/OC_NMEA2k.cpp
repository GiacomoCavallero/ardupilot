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

void nmea2k_updateWind(NMEA2K::WeatherStation& weather, float wind_apparent_speed, float wind_apparent_angle) {
    weather.apparent_wind_angle = wind_apparent_angle;
    weather.apparent_wind_speed = wind_apparent_speed;

    // Filter AWS and AWA
    weather.apparent_wind_angle_filt = weather.filter_awa.filterPoint(weather.apparent_wind_angle);
    weather.apparent_wind_speed_filt = weather.filter_aws.filterPoint(weather.apparent_wind_speed);

    NMEA2K::GPS* gps = nmea2k_sensors.get_active_gps();
    float heading = nmea2k_sensors.get_heading();
    if (gps != NULL && heading >= 0 && heading < 360) {
        {
            // Calculate the True(Water referenced) wind
            Vector2f aw(wind_apparent_speed * cos(ToRad(wind_apparent_angle)), wind_apparent_speed * sin(ToRad(wind_apparent_angle)));
            Vector2f bs(nmea2k_sensors.triducer.longitudinal_speed_water, nmea2k_sensors.triducer.transverse_speed_water);

            Vector2f ww = aw - bs;

            double wws = ww.length();
            double wwa = wrap_180(ToDeg(atan2(ww[1], ww[0])));
            double wwd = wrap_360(wwa + heading);

            weather.water_wind_angle = wwa;
            weather.water_wind_dir = wwd;
            weather.water_wind_speed = wws;

            // Filter TWS and TWD
            weather.water_wind_angle_filt = weather.filter_wwa.filterPoint(weather.water_wind_angle);
            weather.water_wind_dir_filt = weather.filter_wwd.filterPoint(weather.water_wind_dir);
            weather.water_wind_speed_filt = weather.filter_wws.filterPoint(weather.water_wind_speed);
        }
        {
            // Calculate the True(Ground referenced) wind

            // Get apparent wind direction
            float aw_dir = weather.apparent_wind_angle + heading;

            // Wind velocity relative to vehicle in a North-East reference
            Vector2f ab_NE(wind_apparent_speed*cos(ToRad(aw_dir)), wind_apparent_speed*sin(ToRad(aw_dir)));
            // Vehicle ground velocity in North-East reference
            Vector2f bg_NE(gps->sog*cos(ToRad(gps->cog)), gps->sog*sin(ToRad(gps->cog)));
            // Sum to get velocity of air with respect to ground.
            Vector2f ag_NE = ab_NE - bg_NE;

            // Get True(ground referenced) speed and direction.
            double ag_speed = ag_NE.length();
            double ag_dir = ToDeg(atan2(ag_NE[1], ag_NE[0]));
            weather.ground_wind_dir = wrap_360(ag_dir);
            weather.ground_wind_speed = ag_speed;

            // Filter TWS and TWD
            weather.ground_wind_dir_filt = weather.filter_gwd.filterPoint(weather.ground_wind_dir);
            weather.ground_wind_speed_filt = weather.filter_gws.filterPoint(weather.ground_wind_speed);
        }
    }
}

#define GPS_ID                           \
    (pmv->src == gps_primary_id          \
         ? "Primary"                     \
         : (pmv->src == gps_secondary_id \
                ? "Secondary"            \
                : (pmv->src == gps_tertiary_id ? "Tertiary" : "Unknown")))


std::map<uint32_t, uint32_t> new_pgns;

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool NMEA2K::term_complete(unsigned int pgn, MsgVals *pmv)
{
    // printf("Have complete term: %u\n", pgn);
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    uint32_t gps_primary_id = rover.g2.nmea2k.gps_1,
             gps_secondary_id = rover.g2.nmea2k.gps_2,
             gps_tertiary_id = rover.g2.nmea2k.gps_3,
             compass_primary_id = rover.g2.nmea2k.compass_1,
             compass_secondary_id = rover.g2.nmea2k.compass_2,
             water_depth_id = rover.g2.nmea2k.water_depth,
             water_speed_id = rover.g2.nmea2k.water_speed;
//             wind_primary_id = rover.g2.nmea2k.wind_1;
#else
    uint32_t gps_primary_id = NMEA2K_AIRMAR, gps_secondary_id = NMEA2K_AIS,
             gps_tertiary_id = NMEA2K_BACKUP,
             compass_primary_id = NMEA2K_AIRMAR,
             compass_secondary_id = NMEA2K_BACKUP,
             water_depth_id = 255,
             water_speed_id = 255;
//             wind_primary_id = NMEA2K_AIRMAR;
#endif

    GPS *gps_state = NULL;
    if (pmv->src == gps_primary_id)
    {
        gps_state = &primary_gps;
        gps_state->id = gps_primary_id;
    }
    else if (pmv->src == gps_secondary_id)
    {
        gps_state = &secondary_gps;
        gps_state->id = gps_secondary_id;
    }
    else if (pmv->src == gps_tertiary_id)
    {
        gps_state = &tertiary_gps;
        gps_state->id = gps_tertiary_id;
    }
    Compass *compass_state = NULL;
    if (pmv->src == compass_primary_id)
    {
        compass_state = &primary_compass;
        compass_state->id = compass_primary_id;
    }
    else if (pmv->src == compass_secondary_id)
    {
        compass_state = &secondary_compass;
        compass_state->id = compass_secondary_id;
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

    case 126996: //Product Information
        gcs().send_text(MAV_SEVERITY_INFO, "NMEA2K: %d: %s", pmv->src, pmv->getLookup("Model ID"));
        break;

    case 127250: // Vessel Heading
        if (compass_state != NULL)
        {
            // We only want the compass readings from the primary(Airmar) GPS.
            double heading = pmv->getDouble("Heading"); // can be magnetic or true, depending on reference
            int compass_reference = pmv->getInteger("Reference");
//            double declination = pmv->getDouble("Variation");
//            compass_state.variation = ToDeg(declination);
            double variation = get_variation();

            if (fabs(compass_state->roll) > M_PI_4 || fabs(compass_state->pitch) > M_PI_4)
            {
                // If the compass is rolled, or pitched more than 45 degrees, we don't trust it.
            }
            else if (compass_reference == 1)
            {
                // Magnetic
                compass_state->heading = wrap_360(ToDeg(heading + variation));
                compass_state->magnetic = wrap_360(ToDeg(heading));
                compass_state->last_update = AP_HAL::millis64();
                compass_state->heading_filt = compass_state->filter_hdg.filterPoint(compass_state->heading);
            }
//            else
//            {
//                // True
//                primary_compass.heading = wrap_360(ToDeg(heading));
//                compass.magnetic = wrap_360(ToDeg(heading - declination)); // assuming declincation is correct
        }
        break;

    case 127257: // Attitude
        if (compass_state != NULL)
        {
            // We only want the attitude readings from the primary(Airmar) GPS.
            if (pmv->isValid("yaw")) {
                compass_state->yaw = pmv->getDouble("Yaw");
            }
            if (pmv->isValid("Pitch") && pmv->isValid("Roll")) {
                compass_state->pitch = pmv->getDouble("Pitch");
                compass_state->roll = pmv->getDouble("Roll");
            }
        }
        break;

    case 127258: // Magnetic Variation
        if (pmv->isValid("Variation")) {
            double variation = pmv->getDouble("Variation");
            int source = pmv->getInteger("Source");
            // Restricting compass variations sources to WMM 2000 to WMM 2020
            // FIXME: add other WMMs as they are added to PGNs
            if (variation < M_PI && source >= 4 && source <= 8) {
                if (compass_state != NULL) {
                    compass_state->variation = variation;  // Should never be used.
                }
                if (gps_state != NULL) {
                    gps_state->variation = variation;
                }
            }
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
                if (pmv->getInteger("COG Reference") == 0) // 0 True north, 2 Mag north, 3 is bad data
                {
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

                        gps_state->cog_filt = gps_state->filter_cog.filterPoint(gps_state->cog);
                        gps_state->sog_filt = gps_state->filter_sog.filterPoint(gps_state->sog);

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
                    gps_state->num_sats = 6;        // FIXME: A hack
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
                    gps_state->cog_filt = gps_state->filter_cog.filterPoint(gps_state->cog);
                    gps_state->sog_filt = gps_state->filter_sog.filterPoint(gps_state->sog);
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
//        if (pmv->src == wind_primary_id) // Ignore wind data when AIRMAR has no GPS fix
        {
            double wind_speed = pmv->getDouble("Wind Speed");
            double wind_angle = ToDeg(pmv->getDouble("Wind Angle"));
            if (!pmv->isValid("Wind Speed") || !pmv->isValid("Wind Angle") ||
                        wind_speed > 655.34 || wind_speed < 0 || wind_angle < -180 || wind_angle > 360)
            {
                // INVALID readings ignore
            }
            else
            {
                switch (pmv->getInteger("Reference"))
                {
                case 2: // Apparent
                    weather.id = pmv->src;
                    nmea2k_updateWind(weather, wind_speed, wrap_180(wind_angle));
                    break;
                case 0: // True (ground referenced to North)
                case 1: // Magnetic (ground referenced to Magnetic North)
                case 3: // True (boat referenced)
                case 4: // True (water referenced)
                    // For robustness we're only using the apparent wind, which doesn't require a GPS fix, or water speed sensor
                    break;
                default:
                    break;
                }
            }
        }
//        else
//        {
//            // if we have no fix then ignore wind readings
//        }

        break;

    case 128259: // Speed
        if (water_speed_id == 255 || pmv->src == water_speed_id)
        {
            if (fabs(triducer.transverse_speed_water) > FLT_EPSILON)
                triducer.longitudinal_speed_water =
                    pmv->getDouble("Speed Water Referenced");
        }
        break;
    case 130578: // Vessel Speed Components
        if ((water_speed_id == 255 || pmv->src == water_speed_id) &&
                pmv->isValid("Longitudinal Speed, Water-referenced") &&
                pmv->isValid("Transverse Speed, Water-referenced"))
        {
            triducer.id = pmv->src;
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
            triducer.longitudinal_speed_water_filt = triducer.filter_boatspeed.filterPoint(triducer.longitudinal_speed_water);
            triducer.transverse_speed_water_filt = triducer.filter_leeway.filterPoint(triducer.transverse_speed_water);
        }
        break;

    case 128267: // Water Depth
        if (water_depth_id == 255 || pmv->src == water_depth_id)
        {
            triducer.water_depth = pmv->getDouble("Depth");
            triducer.water_offset = pmv->getDouble("Offset");
            triducer.water_range = pmv->getDouble("Range");
        }
        break;

    case 130311: // Environmental Parameters
        if ((water_depth_id == 255 || pmv->src == water_depth_id) &&
                pmv->getInteger("Temperature Source") == 0) // Water temp
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
        // FIXME: This message looks to be filtered so we'll just use the wind from 130306
//        double wind_speed = pmv->getDouble("Wind Speed");
//        double wind_angle = ToDeg(pmv->getDouble("Wind Direction"));
//        double wind_gusts = pmv->getDouble("Wind Gusts");
//        if (!pmv->isValid("Wind Speed") || !pmv->isValid("Wind Direction") ||
//                    wind_speed > 655.34 || wind_speed < 0 || wind_angle < -180 || wind_angle > 360)
//        {
//            // INVALID readings ignore
//        }
//        else
//        {
//            switch (pmv->getInteger("Wind Reference"))
//            {
//            case 2: // Apparent
//                nmea2k_updateWind(weather, wind_speed, wrap_180(wind_angle));
//                break;
//            case 0: // True (ground referenced to North)
//            case 1: // Magnetic (ground referenced to Magnetic North)
//            case 3: // True (boat referenced)
//            case 4: // True (water referenced)
//                // For robustness we're only using the apparent wind, which doesn't require a GPS fix, or water speed sensor
//                break;
//            default:
//                break;
//            }
//        }
//      Currently the AIRMAR doesn't provide a valid gusts reading
//        weather.wind_gusts = wind_gusts;

        if (pmv->isValid("Ambient Temperature")) {
            weather.air_temp = pmv->getDouble("Ambient Temperature");
        }
        if (pmv->isValid("Atmospheric Pressure")) {
            weather.atmos_pressure = pmv->getDouble("Atmospheric Pressure");
        }
    }
    break;

    case 0:      // Ignore
    case 59904:  // ISO Request
    case 60928:  // ISO Address Claim
    case 65410:  // Airmar: Device Information
    case 126993: // Heartbeat
//    case 126996: // Product Information
    case 127251: // Rate of Turn
    case 129044: // Datum
    // case 130311: // Environmental Parameters
    case 130313: // Humidity  // TODO: Does it differ from in 130311?
    case 130314: // Actual Pressure // TODO: Does it differ from in 130311?
                 //        case 130323: // Meteorological Station Data // TODO:
                 //        Does it differ from in 130311?
    case 130316: // Temperature Extended Range
    case 130944: // Airmar: POST
    case 130945: // Fast Packet Transfer
                 // TODO: Look up these PGNs
    case 65408:  // Airmar: Depth Quality Factor
    case 65409:  // Airmar: Speed Pulse Count
    case 128000: // Leeway Angle - TODO: needs pgn.h update
    case 128275: // Distance Log
        break;
    case 129038: // AIS Class A Position Report
    case 129041: // AIS Aids to Navigation (AtoN) Report
    case 129793: // AIS UTC and Date Report
    case 129794: // AIS Class A Static and Voyage Related Data

    case 129809: // AIS Class B static data (msg 24 Part A)
    case 129810: // AIS Class B static data (msg 24 Part B)
    case 130817: // Navico: Product Information
    case 130827: // Lowrance: unknown
    case 130934: // ???
    case 130935: // ???
        break;

    default:
        if (new_pgns.count(pgn) == 0) {
            new_pgns[pgn] = 0;
        }
        uint32_t repeats = new_pgns[pgn];

        if (repeats >= 10) {
            // Limiting to just 10 repeats, to avoid flooding.
        } else {
            // update repeat count
            new_pgns[pgn] = ++repeats;

            // Look up description.
            const char* pgn_desc = NULL;
            for (Pgn* pgn_info = pgnListFirst(); pgn_info != pgnListEnd(); ++pgn_info) {
                if (pgn_info->pgn == pgn) {
                    pgn_desc = pgn_info->description;
                }
            }

            if (pgn_desc != NULL) {
                gcs().send_text(MAV_SEVERITY_DEBUG,
                                "Ocius N2K: %u sent pgn %u(%s).", pmv->src, pgn, pgn_desc);
            } else {
                gcs().send_text(MAV_SEVERITY_DEBUG,
                                "Ocius N2K: %u sent unknown pgn %u.", pmv->src, pgn);
            }
        }
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

Location NMEA2K::get_location() {
    GPS* gps = get_active_gps();
    if (gps != NULL)
        return gps->location;
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    Location l;
    if (rover.ahrs.get_position(l)) {
        return l;
    }
#endif
    return Location();
}

float NMEA2K::get_heading() {
    // FIXME: return the filtered value
    Compass* compass = get_active_compass();
    if (compass != NULL) {
        return compass->heading;
    }
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    return wrap_360(ToDeg(rover.ahrs.get_yaw()));
#else
    return 0;
#endif
}

bool NMEA2K::commsMastDown() {
    // FIXME need a proper way to determine which compass is the airmar
    if (primary_compass.id >= 30 && primary_compass.id <= 40) {
        return fabs(primary_compass.pitch) > M_PI_4 || fabs(primary_compass.roll) > M_PI_4;
    } else if (secondary_compass.id >= 30 && secondary_compass.id <= 40) {
        return fabs(secondary_compass.pitch) > M_PI_4 || fabs(secondary_compass.roll) > M_PI_4;
    }
    return false;
}
