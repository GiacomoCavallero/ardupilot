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

#include "AP_WindVane_Ocius.h"
#include <OC_NMEA2k/OC_NMEA2k.h>
#include <AP_AHRS/AP_AHRS.h>

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
#include <../APMrover2/Rover.h>
#endif

// constructor
AP_WindVane_Ocius::AP_WindVane_Ocius(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
    nmea2k_sensors.init();
}

void AP_WindVane_Ocius::update_direction()
{
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    if (rover.g2.nmea2k.use_filtered) {
        update_frontend_all(wrap_2PI(ToRad(nmea2k_sensors.weather.ground_wind_dir_filt)), nmea2k_sensors.weather.ground_wind_speed_filt,
                wrap_PI(ToRad(nmea2k_sensors.weather.apparent_wind_angle_filt)), nmea2k_sensors.weather.apparent_wind_speed_filt);
    } else {
        update_frontend_all(wrap_2PI(ToRad(nmea2k_sensors.weather.ground_wind_dir)), nmea2k_sensors.weather.ground_wind_speed,
                        wrap_PI(ToRad(nmea2k_sensors.weather.apparent_wind_angle)), nmea2k_sensors.weather.apparent_wind_speed);
    }
#else
    update_frontend_all(wrap_2PI(ToRad(nmea2k_sensors.weather.ground_wind_dir)), nmea2k_sensors.weather.ground_wind_speed,
                    wrap_PI(ToRad(nmea2k_sensors.weather.apparent_wind_angle)), nmea2k_sensors.weather.apparent_wind_speed);
#endif
}

void AP_WindVane_Ocius::update_speed()
{
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    if (rover.g2.nmea2k.use_filtered) {
        update_frontend_all(wrap_2PI(ToRad(nmea2k_sensors.weather.ground_wind_dir_filt)), nmea2k_sensors.weather.ground_wind_speed_filt,
                wrap_PI(ToRad(nmea2k_sensors.weather.apparent_wind_angle_filt)), nmea2k_sensors.weather.apparent_wind_speed_filt);
    } else {
        update_frontend_all(wrap_2PI(ToRad(nmea2k_sensors.weather.ground_wind_dir)), nmea2k_sensors.weather.ground_wind_speed,
                        wrap_PI(ToRad(nmea2k_sensors.weather.apparent_wind_angle)), nmea2k_sensors.weather.apparent_wind_speed);
    }
#else
    update_frontend_all(wrap_2PI(ToRad(nmea2k_sensors.weather.ground_wind_dir)), nmea2k_sensors.weather.ground_wind_speed,
                    wrap_PI(ToRad(nmea2k_sensors.weather.apparent_wind_angle)), nmea2k_sensors.weather.apparent_wind_speed);
#endif
}
