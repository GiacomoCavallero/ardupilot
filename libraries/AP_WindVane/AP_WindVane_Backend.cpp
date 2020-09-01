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

#include "AP_WindVane.h"
#include "AP_WindVane_Backend.h"

// base class constructor.
AP_WindVane_Backend::AP_WindVane_Backend(AP_WindVane &frontend) :
        _frontend(frontend)
{
}

// update speed to frontend
void AP_WindVane_Backend::speed_update_frontend(float apparent_speed_in)
{
    // apply low pass filter if enabled
    if (is_positive(_frontend._speed_filt_hz)) {
        _speed_filt.set_cutoff_frequency(_frontend._speed_filt_hz);
        _frontend._speed_apparent = _speed_filt.apply(apparent_speed_in, 0.02f);
    } else {
        _frontend._speed_apparent = apparent_speed_in;
    }
}

// update speed to frontend
void AP_WindVane_Backend::speed_true_update_frontend(float true_speed_in)
{
    // apply low pass filter if enabled
    _frontend._sensor_reference = _frontend.WindReference::WIND_TRUE;
    if (is_positive(_frontend._speed_filt_hz)) {
        _speed_filt.set_cutoff_frequency(_frontend._speed_filt_hz);
        _frontend._speed_true = _speed_filt.apply(true_speed_in, 0.02f);
    } else {
        _frontend._speed_true = true_speed_in;
    }
}

// update direction to frontend
void AP_WindVane_Backend::direction_update_frontend(float apparent_direction_ef)
{
    // apply low pass filter if enabled
    if (is_positive(_frontend._dir_filt_hz)) {
        _dir_sin_filt.set_cutoff_frequency(_frontend._dir_filt_hz);
        _dir_cos_filt.set_cutoff_frequency(_frontend._dir_filt_hz);
        // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
        const float filtered_sin = _dir_sin_filt.apply(sinf(apparent_direction_ef), 0.05f);
        const float filtered_cos = _dir_cos_filt.apply(cosf(apparent_direction_ef), 0.05f);
        _frontend._direction_apparent_ef = wrap_PI(atan2f(filtered_sin, filtered_cos));
    } else {
        _frontend._direction_apparent_ef = wrap_PI(apparent_direction_ef);
    }

    // apply low pass filter for current tack, this is at a hard coded cutoff frequency
    const float tack_direction_apparent = _tack_filt.apply(wrap_PI(apparent_direction_ef - AP::ahrs().yaw), 0.05f);
    _frontend._current_tack = is_negative(tack_direction_apparent) ? _frontend.Sailboat_Tack::TACK_PORT : _frontend.Sailboat_Tack::TACK_STARBOARD;
}

// update direction to frontend
void AP_WindVane_Backend::direction_true_update_frontend(float true_angle_ef)
{
    // apply low pass filter if enabled
    _frontend._sensor_reference = _frontend.WindReference::WIND_TRUE;
    if (is_positive(_frontend._dir_filt_hz)) {
        _dir_sin_filt.set_cutoff_frequency(_frontend._dir_filt_hz);
        _dir_cos_filt.set_cutoff_frequency(_frontend._dir_filt_hz);
        // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
        const float filtered_sin = _dir_sin_filt.apply(sinf(true_angle_ef), 0.05f);
        const float filtered_cos = _dir_cos_filt.apply(cosf(true_angle_ef), 0.05f);
        _frontend._direction_true = wrap_2PI(atan2f(filtered_sin, filtered_cos));
    } else {
        _frontend._direction_true = wrap_2PI(true_angle_ef);
    }

    // apply low pass filter for current tack, this is at a hard coded cutoff frequency
    // FIXME: MM: Shouldn't be in wind vane, but do it anyway
    const float tack_direction_apparent = _tack_filt.apply(wrap_PI(true_angle_ef - AP::ahrs().yaw), 0.05f);
    _frontend._current_tack = is_negative(tack_direction_apparent) ? _frontend.Sailboat_Tack::TACK_PORT : _frontend.Sailboat_Tack::TACK_STARBOARD;
}

void AP_WindVane_Backend::update_frontend_all(float true_direction_ef, float true_speed_in, float apparent_angle_bf, float apparent_speed_in) {
    _frontend._sensor_reference = _frontend.WindReference::WIND_BOTH;

    _frontend._direction_true = wrap_2PI(true_direction_ef);
    _frontend._speed_true = true_speed_in;
    // FIXME: MM: I assume we're using the apparent direction because it is assumed the wind updates much slower than the compass heading
    _frontend._direction_apparent_ef = wrap_PI(apparent_angle_bf + AP::ahrs().yaw);
    _frontend._speed_apparent = apparent_speed_in;

    // FIXME: MM: Shouldn't be in wind vane, but do it anyway
//    const float tack_direction_apparent = _tack_filt.apply(apparent_angle_bf, 0.05f);
    _frontend._current_tack = is_negative(apparent_angle_bf) ? _frontend.Sailboat_Tack::TACK_PORT : _frontend.Sailboat_Tack::TACK_STARBOARD;
}

// calibrate WindVane
void AP_WindVane_Backend::calibrate()
{
    gcs().send_text(MAV_SEVERITY_INFO, "WindVane: No cal required");
    _frontend._calibration.set_and_save(0);
    return;
}
