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

// constructor
AP_WindVane_Ocius::AP_WindVane_Ocius(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
    nmea2k_sensors.init();
}

void AP_WindVane_Ocius::update_direction()
{
    // temporarily store true speed and direction for easy access
//    const float wind_speed = nmea2k_sensors.weather.wind_average.speed;
//    const float wind_dir_rad = radians(nmea2k_sensors.weather.wind_average.direction);

    direction_true_update_frontend(radians(nmea2k_sensors.weather.wind_dir_true));

//    // Note than the SITL wind direction is defined as the direction the wind is traveling to
//    // This is accounted for in these calculations
//
//    // convert true wind speed and direction into a 2D vector
//    Vector2f wind_vector_ef(cosf(wind_dir_rad) * wind_speed, sinf(wind_dir_rad) * wind_speed);
//
//    Vector3f ground_speed;
//    if(AP::ahrs().get_velocity_NED(ground_speed)) {
//        // add vehicle speed to get apparent wind vector
//        wind_vector_ef.x += ground_speed.x;
//        wind_vector_ef.y += ground_speed.y;
//    }
//
//    direction_update_frontend(atan2f(wind_vector_ef.y, wind_vector_ef.x));
}

void AP_WindVane_Ocius::update_speed()
{
    // temporarily store true speed and direction for easy access
//    const float wind_speed = nmea2k_sensors.weather.wind_speed_true;
//    const float wind_dir_rad = radians(nmea2k_sensors.weather.wind_dir_true);

    speed_true_update_frontend(nmea2k_sensors.weather.wind_speed_true);

//    // convert true wind speed and direction into a 2D vector
//    Vector2f wind_vector_ef(cosf(wind_dir_rad) * wind_speed, sinf(wind_dir_rad) * wind_speed);
//
//    Vector3f ground_speed;
//    if(AP::ahrs().get_velocity_NED(ground_speed)) {
//        // add vehicle speed to get apparent wind vector
//        wind_vector_ef.x += ground_speed.x;
//        wind_vector_ef.y += ground_speed.y;
//    }
//
//    speed_update_frontend(wind_vector_ef.length());
}
