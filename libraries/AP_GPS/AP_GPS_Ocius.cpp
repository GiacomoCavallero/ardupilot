// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//

/// @file	AP_GPS_AIRMARN2K.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include "AP_GPS_Ocius.h"

#include <AP_Common/AP_Common.h>

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <GCS_MAVLink/GCS.h>
#include <OC_NMEA2k/OC_NMEA2k.h>

extern const AP_HAL::HAL& hal;

AP_GPS_Ocius::AP_GPS_Ocius(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    nmea2k_sensors.init(*port);

    gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K GPS Driver Rebooted.");

//    port = new AP_HAL::UARTDriver(false);
}

bool AP_GPS_Ocius::read(void)
{
    nmea2k_sensors.read(*port);

    // copy data into state
    NMEA2K::GPS *nmea_gps = &nmea2k_sensors.primary_gps;
    AP_GPS::GPS_Status nmea_status = AP_GPS::GPS_OK_FIX_3D;
    if (!nmea_gps->have_fix){
        if (nmea2k_sensors.secondary_gps.have_fix) {
            nmea_gps = &nmea2k_sensors.secondary_gps;
            nmea_status = AP_GPS::GPS_OK_FIX_2D;
        } else if (nmea2k_sensors.tertiary_gps.have_fix) {
            nmea_gps = &nmea2k_sensors.tertiary_gps;
            nmea_status = AP_GPS::GPS_OK_FIX_2D;
        } else {
            nmea_status = AP_GPS::NO_FIX;
        }
    }

    state.status = nmea_status;
    state.location = nmea_gps->location;
    state.ground_course = nmea_gps->cog;
    state.ground_speed = nmea_gps->sog;
    state.hdop = nmea_gps->hdop;
    state.vdop = nmea_gps->vdop;
    state.num_sats = nmea_gps->num_sats;
    state.time_week = nmea_gps->time_week;
    state.time_week_ms = nmea_gps->time_week_ms;
    state.last_gps_time_ms = nmea_gps->time_last_update;

    fill_3d_velocity();

    return false;
}

/*
  detect a NMEA GPS. Adds one byte, and returns true if the stream
  matches a NMEA string
 */
bool
AP_GPS_Ocius::_detect(struct NMEA_detect_state &state, uint8_t data)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Ocius N2K GPS detecting.");
    return false;
}
