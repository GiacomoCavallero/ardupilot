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

#include "AP_WindVane_SITL.h"

// constructor
AP_WindVane_SITL::AP_WindVane_SITL(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

void AP_WindVane_SITL::update_direction()
{
    direction_true_update_frontend(radians(AP::sitl()->wind_direction_active));
}

void AP_WindVane_SITL::update_speed()
{
    speed_true_update_frontend(AP::sitl()->wind_speed_active);
}
#endif
