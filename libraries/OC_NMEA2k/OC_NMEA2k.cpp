/*
 * OC_NMEA2k.cpp
 *
 *  Created on: 16 Dec. 2019
 *      Author: mmcgill
 */

#include <OC_NMEA2k/OC_NMEA2k.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/UARTDriver.h>
#include <GCS_MAVLink/GCS.h>
#include "n2kparse.h"
#include <stdio.h>

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
#include <../APMrover2/Rover.h>
#endif

NMEA2K nmea2k_sensors;

#define NMEA2K_AIRMAR       36
#define NMEA2K_AIS          43
#define NMEA2K_SIMRAD       37 // To be set to ID of new water sensor
#define NMEA2K_BACKUP       127 // new gps/compass on foredeck ignore for now

#define NMEA2K_LATLONG_RESOLUTION  10000000

static uint32_t getBSDDate(const char* date) {
    uint32_t year, month, day;
    sscanf(date, "%u.%u.%u", &year, &month, &day);
    uint32_t rval = day * 10000 + month * 100 + (year % 100);
    return rval;
}

static uint32_t getHMSInMillis(const char* time) {
    int32_t hour, minute;
    float second;
    sscanf(time, "%u:%u:%f", &hour, &minute, &second);
    uint32_t rval = hour * 10000000 + minute * 100000 + second * 1000;
    return rval;
}

static void
nmea2k_writeMessage(AP_HAL::UARTDriver& port, unsigned char command, const unsigned char * cmd, const int len)
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
static void nmea2k_make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds, NMEA2K::GPS& state)
{
    uint8_t year, mon, day, hour, min, sec;
    uint16_t msec;

    year = bcd_date % 100;
    mon  = (bcd_date / 100) % 100;
    day  = bcd_date / 10000;

    uint32_t v = bcd_milliseconds;
    msec = v % 1000; v /= 1000;
    sec  = v % 100; v /= 100;
    min  = v % 100; v /= 100;
    hour = v % 100;

    int8_t rmon = mon - 2;
    if (0 >= rmon) {
        rmon += 12;
        year -= 1;
    }

    // get time in seconds since unix epoch
    uint32_t ret = (year/4) - (GPS_LEAPSECONDS_MILLIS / 1000UL) + 367*rmon/12 + day;
    ret += year*365 + 10501;
    ret = ret*24 + hour;
    ret = ret*60 + min;
    ret = ret*60 + sec;

    // convert to time since GPS epoch
    ret -= 272764785UL;

    // get GPS week and time
    state.time_week = ret / AP_SEC_PER_WEEK;
    state.time_week_ms = (ret % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC;
    state.time_week_ms += msec;
}

#define GPS_ID  (pmv->src == gps_primary_id?"Primary":                          \
                    (pmv->src == gps_secondary_id?"Secondary":                  \
                        (pmv->src == gps_tertiary_id?"Tertiary":"Unknown")))

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool NMEA2K::term_complete(unsigned int pgn, MsgVals *pmv)
{
    printf("Have complete term: %u\n", pgn);
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    uint32_t gps_primary_id = rover.g2.nmea2k.gps_1,
            gps_secondary_id = rover.g2.nmea2k.gps_2,
            gps_tertiary_id = rover.g2.nmea2k.gps_3;
#else
    uint32_t gps_primary_id = NMEA2K_AIRMAR,
            gps_secondary_id = NMEA2K_AIS,
            gps_tertiary_id = NMEA2K_BACKUP;
#endif

    GPS* gps_state = NULL;
    if (pmv->src == gps_primary_id) {
        gps_state = &primary_gps;
    } else if (pmv->src == gps_secondary_id) {
        gps_state = &secondary_gps;
    } else if (pmv->src == gps_tertiary_id) {
        gps_state = &tertiary_gps;
    }

    char method[64];
    double mag;
    double declination;
    char own_info[64];
//    float windDir;

//    if (pmv->src == NMEA2K_BACKUP) // FIXME: backup gps ignore for now
//      return false;

    switch(pgn) {
        case 126992: // System Time
            if (gps_state != NULL && gps_state->have_fix) {
                nmea2k_make_gps_time(getBSDDate(pmv->getDateOrTime("Date")), getHMSInMillis(pmv->getDateOrTime("Time")), *gps_state);
//                secondary_gps_state.time_week = gps_state->time_week;
//                secondary_gps_state.time_week_ms = gps_state->time_week_ms;
                gps_state->time_last_update = AP_HAL::millis64();
            }
            break;

        case 127250: // Vessel Heading
            // TODO: check heading reference before adding
            mag = pmv->getDouble("Heading");
            declination = pmv->getDouble("Variation");
//            compass.heading = ToDeg(mag);
            compass.reference = pmv->getInteger("Reference");
            compass.variation = ToDeg(declination);

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
            compass.offset = rover.g2.magnetic_offset;
            if (compass.reference) {
                compass.heading = CLIP_360(ToDeg(mag + declination) + rover.g2.magnetic_offset);
            } else {
                compass.heading = CLIP_360(ToDeg(mag) + rover.g2.magnetic_offset);
            }
#else
            if (compass.reference) {
                compass.heading = CLIP_360(ToDeg(mag + declination));
            } else {
                compass.heading = CLIP_360(ToDeg(mag));
            }
#endif
            compass.last_update = AP_HAL::micros64();
            break;

        case  129025:
            //Position Rapid Update
            if (gps_state != NULL)
            {
                if (pmv->isValid("Latitude") && pmv->isValid("Longitude"))
                {
                    if (fabs(pmv->getDouble("Latitude")) > 0 && fabs(pmv->getDouble("Longitude")) > 0)
                    {
                        gps_state->location.lat     = pmv->getDouble("Latitude") * NMEA2K_LATLONG_RESOLUTION;
                        gps_state->location.lng     = pmv->getDouble("Longitude") * NMEA2K_LATLONG_RESOLUTION;
                        if (!gps_state->have_fix)
                            gcs().send_text(MAV_SEVERITY_INFO, "Ocius N2K: GPS Fix(%s) restored from msg %d", GPS_ID, pgn);
                        gps_state->have_fix = true;
                        gps_state->last_update = AP_HAL::millis64();
                    }
                    else
                    {
                        if (gps_state->have_fix)
                            gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K: No GPS FIX(%s) from msg %d", GPS_ID, pgn);
                        gps_state->have_fix = false;
                    }
                }
                else
                {
                    if (gps_state->have_fix)
                        gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K: No GPS FIX(%s) from msg %d", GPS_ID, pgn);
                    gps_state->have_fix = false;
                }
            }
            break;

        case  129026:
            //COG SOG Rapid Update
            if (gps_state != NULL) //AIRMAR
            {
                if (pmv->isValid("SOG") && pmv->isValid("COG"))
                {
                    if (pmv->getInteger("COG Reference") == 0) //0 True north, 2 Mag north, 3 is bad data
                    {
                        //state.ground_speed     = pmv->getDouble("SOG");
                        //state.ground_course = ToDeg(pmv->getDouble("COG"));
                        double gs = pmv->getDouble("SOG");
                        double gc = ToDeg(pmv->getDouble("COG"));
                        gps_state->cog = static_cast<float>(gc);
                        gps_state->sog = static_cast<float>(gs);

                        // TODO: Do we want to average the cog/sog again?

//                        gps_state->average.push_reading(gc, gs);
//                        gps_state->cog  = gps_state->average.speed;
//                        gps_state->sog = gps_state->average.direction;

                        gps_state->last_update = AP_HAL::micros64();
                    }
                }
            }
            break;

        case  129029:
            //GNSS Position Data
            strcpy(method, pmv->getLookup("Method"));
            if (gps_state != NULL) {
                 if (!strcmp(method, "no GNSS"))
                 {
                     if (gps_state->have_fix)
                         gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K: No GNSS FIX(%s) from msg %d", GPS_ID, pgn);
                     gps_state->have_fix = false;
                 }
                 else
                 {
                    if (!gps_state->have_fix)
                        gcs().send_text(MAV_SEVERITY_INFO, "Ocius N2K: GNSS Fix(%s) restored from msg %d", GPS_ID, pgn);
                    gps_state->have_fix = true;
                    gps_state->location.lat     = pmv->getDouble("Latitude") * NMEA2K_LATLONG_RESOLUTION;
                    gps_state->location.lng     = pmv->getDouble("Longitude") * NMEA2K_LATLONG_RESOLUTION;
                    gps_state->location.alt     = pmv->getDouble("Altitude") * 100;
                    gps_state->hdop             = pmv->getDouble("HDOP") * 100;
                    nmea2k_make_gps_time(getBSDDate(pmv->getDateOrTime("Date")), getHMSInMillis(pmv->getDateOrTime("Time")), *gps_state);
                    gps_state->last_update = AP_HAL::millis64();
                    gps_state->num_sats     = pmv->getInteger("Number of SVs");
                  }
            }
            break;

        case 129033: // Time and Date
            if (gps_state != NULL && gps_state->have_fix) {
                nmea2k_make_gps_time(getBSDDate(pmv->getDateOrTime("Date")), getHMSInMillis(pmv->getDateOrTime("Time")), *gps_state);
                gps_state->last_update = AP_HAL::millis64();
            }
            break;

        case  129039:
            // AIS Class B Position Report - includes self report from AIS
            if (gps_state != NULL) {
                strcpy(own_info, pmv->getLookup("AIS Transceiver information"));
                if (!strncmp(own_info, "Own information", 15))
                {
                    if (pmv->isValid("Latitude") && pmv->isValid("Longitude"))
                    {
    //                    ais_gps_state.last_gps_time_ms = AP_HAL::millis();
                        gps_state->location.lat     = pmv->getDouble("Latitude") * NMEA2K_LATLONG_RESOLUTION;
                        gps_state->location.lng     = pmv->getDouble("Longitude") * NMEA2K_LATLONG_RESOLUTION;
                        gps_state->location.alt     = 0;
                        if (!gps_state->have_fix)
                            gcs().send_text(MAV_SEVERITY_INFO, "Ocius N2K: AIS GPS Fix(%s) restored from msg %d", GPS_ID, pgn);
                        gps_state->have_fix = true;
                        gps_state->last_update = AP_HAL::millis64();
                    }
                    else
                    {
                        if (gps_state->have_fix)
                            gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K: No AIS GPS FIX(%s) from msg %d", GPS_ID, pgn);
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
//                        gps_state->cog  = gps_state->average.speed;
//                        gps_state->sog = gps_state->average.direction;
                    }
                }
            }
            break;

        case  129539:
            //GNSS DOPs
            if (gps_state != NULL) {
                if (gps_state->have_fix) {
                    gps_state->hdop         = pmv->getDouble("HDOP") * 100;
                    gps_state->vdop         = pmv->getDouble("VDOP") * 100;
                }
            }
            break;

        case  129540:
            //GNSS Sats In View
//          airmar_gps_state.num_sats      = pmv->getInteger("Sats in View");
//          gcs().send_text(MAV_SEVERITY_WARNING, "Airmar N2K: Sats in view %d", pmv->getInteger("Sats in View"));
//            gpsUpdated = true;
            break;

        case 130306: // Wind Data
            if (primary_gps.have_fix)
            {
                switch (pmv->getInteger("Reference"))
                {
                    case 0: // True wind
                        weather.wind_dir_true = ToDeg(pmv->getDouble("Wind Angle"));
                        weather.wind_speed_true = pmv->getDouble("Wind Speed");
                        break;
                    case 1: // Magnetic
                    case 2: // Apparent
                    case 3: // True (boat referenced)
                    case 4: // True (water referenced)
                        // TODO: consider use of other readings
                    default:
                        break;
                }

                // TODO: Do we wish to average the wind speed/direction?
            }
            else
            {
                // if we have no fix then ignore wind readings
            }

            break;

        case 128259: // Speed
            triducer.speed_thru_water = pmv->getDouble("Speed Water Referenced");
            break;

        case 128267: // Water Depth
            triducer.water_depth = pmv->getDouble("Depth");
            break;

        case 130311: // Temperature
            if (pmv->getInteger("Temperature Source") == 0)
            {
                triducer.water_temp = pmv->getDouble("Temperature");
            }
            break;
        case 130312: // Temperature
        //    if (pmv->getInteger("Temperature Source") == 0)
        //    {
        //        triducer.water_temp = pmv->getDouble("Actual Temperature");
        //    }
            break;

        case 59904:  // ISO Request
        case 60928:  // ISO Address Claim
        case 65410:  // Airmar: Device Information
        case 127251: // Rate of Turn
        case 127257: // Attitude
        case 127258: // Magnetic Variation
        case 129044: // Datum
        //case 130311: // Environmental Parameters
        case 130313: // Humidity
        case 130314: // Actual Pressure
        case 130323: // Meteorological Station Data
        case 130944: // Airmar: POST
        case 0: // Ignore
        case 130945: // Fast Packet Transfer
            // TODO: Look up these PGNs
        case 65408:  // Airmar: Depth Quality Factor
        case 65409:  // Airmar: ???
        case 128275: // Distance Log
            break;
        case 129038: // AIS Class A Position Report
        case 129794: // AIS Class A Static and Voyage Related Data

    case 129809: // AIS Class B static data (msg 24 Part A)
    case 129810: // AIS Class B static data (msg 24 Part B)
    case 130934:
    case 130935:
    case 130578: //new water speed sensor
            break;
        default:
                gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K received pgn %d(0x%x)", pgn, pgn);
            break;
    }

    //Check GPS Status here
    {
        uint32_t tnow = AP_HAL::millis64();
        uint32_t tdiff = tnow - primary_gps.last_update;
        if (tdiff > 5000) {
            //we have not had an airmar message for 5 seconds, it may be powered off
            //they should come every 2 seconds from the GME ais unit
            if (primary_gps.have_fix) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K: GPS(Primary) TIMEOUT, no GPS update in 5s");
                primary_gps.have_fix = false;
            }
        }
        tdiff = tnow - secondary_gps.last_update;
        if (tdiff > 5000) {
            //we have not had an airmar message for 5 seconds, it may be powered off
            //they should come every 2 seconds from the GME ais unit
            if (secondary_gps.have_fix) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K: GPS(Secondary) TIMEOUT, no GPS update in 5s");
                secondary_gps.have_fix = false;
            }
        }
        tdiff = tnow - tertiary_gps.last_update;
        if (tdiff > 5000) {
            //we have not had an airmar message for 5 seconds, it may be powered off
            //they should come every 2 seconds from the GME ais unit
            if (tertiary_gps.have_fix) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K: GPS(Tertiary) TIMEOUT, no GPS update in 5s");
                tertiary_gps.have_fix = false;
            }
        }
    }
    return false;
}

void NMEA2K::init(AP_HAL::UARTDriver& port) {
    nmea2k_writeMessage(port, NGT_MSG_SEND, NGT_STARTUP_SEQ, sizeof(NGT_STARTUP_SEQ));
}

bool NMEA2K::read(AP_HAL::UARTDriver& port) {
    bool parsed = false;
    int16_t numc = port.available();
    if (numc == 0) {
        return false;
    }

    while (numc) {
        unsigned char c = port.read();
        int len = readNGT1Byte(c, msg);
        if (len)
        {
            //we have a complete NGT message
            int command = 0;
            int payLen = 0;
            int mtype = messageReceived(msg, len, command, payLen);
            if (mtype == 1)
            {
                //n2k message
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


