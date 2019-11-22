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
/*
    Sailboat simulator class

    see explanation of lift and drag explained here: https://en.wikipedia.org/wiki/Forces_on_sails

    To-Do: add heel handling by calculating lateral force from wind vs gravity force from heel to arrive at roll rate or acceleration
*/

#include "SIM_Ocius.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

//#include <APMrover2/Rover.h>

#include <AP_HAL_SITL/RCOutput.h>

#include <string.h>
#include <stdio.h>

#define BLUEBOTTLE_FORCE_REDUCTION      0.122F

extern const AP_HAL::HAL& hal;

namespace SITL {

#define STEERING_SERVO_CH   0   // steering controlled by servo output 1
#define THROTTLE_SERVO_CH   2   // throttle controlled by servo output 3

#define MAST_RAISE_CH       8   // mast raise/lower controlled by servo output 9
#define SAIL_ROTATE_CH      9   // sail rotate controlled by servo output 10

    // very roughly sort of a stability factors for waves
#define WAVE_ANGLE_GAIN 1
#define WAVE_HEAVE_GAIN 1

SimOcius::SimOcius(const char *frame_str) :
    Aircraft(frame_str),
    steering_angle_max(35),
    turning_circle(1.8),
    sail_area(0.0),
    wave_heave(0),
    wave_phase(0),
    rudder(0),
    mast(-1,0.3f),
    sail(0,0.3f)
{
    wamv = (strcmp(frame_str, "wamv") == 0);
}

// calculate the lift and drag as values from 0 to 1
// given an apparent wind speed in m/s and angle-of-attack in degrees
void SimOcius::calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const
{
    const float sail_camber = 5;
    const uint16_t index_width_deg = 10;
    const uint8_t index_max = ARRAY_SIZE(lift_curve) - 1;

    float aoa_drag = fabs(angle_of_attack_deg);
    // check extremes
    if (aoa_drag <= 0.0f) {
        drag = drag_curve[0];
    } else if (aoa_drag >= index_max * index_width_deg) {
        drag = drag_curve[index_max];
    } else {
        uint8_t index = constrain_int16(aoa_drag / index_width_deg, 0, index_max);
        float remainder = aoa_drag - (index * index_width_deg);
        drag = linear_interpolate(drag_curve[index], drag_curve[index+1], remainder, 0.0f, index_width_deg);
    }

    float aoa_lift = angle_of_attack_deg + sail_camber;
    bool lift_is_neg = aoa_lift < 0.0f;
    aoa_lift = fabs(aoa_lift);

    // check extremes
    // TODO: angle of attack is negative, do we have negative lift??
    if (aoa_lift <= 0.0f) {
        lift = lift_curve[0];
    } else if (aoa_lift >= index_max * index_width_deg) {
        lift = lift_curve[index_max];
    } else {
        uint8_t index = constrain_int16(aoa_lift / index_width_deg, 0, index_max);
        float remainder = aoa_lift - (index * index_width_deg);
        lift = linear_interpolate(lift_curve[index], lift_curve[index+1], remainder, 0.0f, index_width_deg);
    }
    if (lift_is_neg) lift = -lift;

    // apply scaling by wind speed
    lift *= wind_speed * sail_area;
    drag *= wind_speed * sail_area;
}

// return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
float SimOcius::get_turn_circle(float steering) const
{
    if (is_zero(steering)) {
        return 0;
    }
    return turning_circle * sinf(radians(steering_angle_max)) / sinf(radians(steering * steering_angle_max));
}

// return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
float SimOcius::get_yaw_rate(float steering, float speed) const
{
    if (is_zero(steering) || is_zero(speed)) {
        return 0;
    }
    float d = get_turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}

// return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
float SimOcius::get_lat_accel(float steering, float speed) const
{
    float yaw_rate = get_yaw_rate(steering, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

// simulate basic waves / swell
void SimOcius::update_wave(float delta_time)
{
    const float wave_heading = sitl->wave.direction;
    const float wave_speed = sitl->wave.speed;
    const float wave_lenght = sitl->wave.length;
    const float wave_amp = sitl->wave.amp;

    // apply rate propositional to error between boat angle and water angle
    // this gives a 'stability' effect
    float r, p, y;
    dcm.to_euler(&r, &p, &y); 

    // if not armed don't do waves, to allow gyro init
    if (sitl->wave.enable == 0 || !hal.util->get_soft_armed() || is_zero(wave_amp) ) { 
        wave_gyro = Vector3f(-r,-p,0.0f) * WAVE_ANGLE_GAIN;
        wave_heave = -velocity_ef.z * WAVE_HEAVE_GAIN;
        wave_phase = 0.0f;
        return;
    }

    // calculate the sailboat speed in the direction of the wave
    const float boat_speed = velocity_ef.x * sinf(radians(wave_heading)) + velocity_ef.y * cosf(radians(wave_heading));

    // update the wave phase
    const float aprarent_wave_distance = (wave_speed - boat_speed) * delta_time;
    const float apparent_wave_phase_change = (aprarent_wave_distance / wave_lenght) * M_2PI;

    wave_phase += apparent_wave_phase_change;
    wave_phase = wrap_2PI(wave_phase);

    // calculate the angles at this phase on the wave
    // use basic sine wave, dy/dx of sine = cosine
    // atan( cosine ) = wave angle
    const float wave_slope = (wave_amp * 0.5f) * (M_2PI / wave_lenght) * cosf(wave_phase);
    const float wave_angle = atanf(wave_slope);

    // convert wave angle to vehicle frame
    const float heading_dif = wave_heading - y;
    float angle_error_x = (sinf(heading_dif) * wave_angle) - r;
    float angle_error_y = (cosf(heading_dif) * wave_angle) - p;

    // apply gain
    wave_gyro.x = angle_error_x * WAVE_ANGLE_GAIN;
    wave_gyro.y = angle_error_y * WAVE_ANGLE_GAIN;
    wave_gyro.z = 0.0f;

    // calculate wave height (NED)
    if (sitl->wave.enable == 2) {
        wave_heave = (wave_slope - velocity_ef.z) * WAVE_HEAVE_GAIN;
    } else {
        wave_heave = 0.0f;
    }
}

// return the angle of attack for the sail
float SimOcius::get_angle_of_attack(float wind_apparent_dir_bf){
    float angle_of_attack = 0;

    float sail_angle = sail.get_position() * 90;

    if (wind_apparent_dir_bf > 0) {
        angle_of_attack = wind_apparent_dir_bf - 90 - sail_angle;
    } else {
        angle_of_attack = -wind_apparent_dir_bf - 90 + sail_angle;
    }

    return angle_of_attack;
}

/*
  update the sailboat simulation by one time step
 */
void SimOcius::update(const struct sitl_input &input)
{
    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // update wind
    update_wind(input);

    float steering = 0.0f;

    float throttle_force = 0.0f;
    if (wamv) {
        // FIXME: the Wam-V uses skid steering
    } else {
        // the steering controls the rudder, the throttle controls the main sail position
        steering = -2*((input.servos[STEERING_SERVO_CH]-1000)/1000.0f - 0.5f);
        rudder.set_desired_position(steering);
        rudder.update(delta_time);
        steering = rudder.get_position();

        // throttle force (for motor sailing)
        // gives throttle force == hull drag at 10m/s
        const uint16_t throttle_out = constrain_int16(input.servos[THROTTLE_SERVO_CH], 1000, 2000);
        throttle_force = -(throttle_out-1500) * 0.1f * BLUEBOTTLE_FORCE_REDUCTION;

        if (input.servos[MAST_RAISE_CH] >= 1000 && input.servos[MAST_RAISE_CH] <= 2000) {
            float mast_set_pos = 2*((input.servos[MAST_RAISE_CH]-1000)/1000.0f - 0.5f);
            mast.set_desired_position(mast_set_pos);
        }
        mast.update(delta_time);

        sail_area = sin(M_PI * (mast.get_position() + 1) / 4);

        if (input.servos[SAIL_ROTATE_CH] >= 1000 && input.servos[SAIL_ROTATE_CH] <= 2000) {
            float sail_set_pos = 2*((input.servos[SAIL_ROTATE_CH]-1000)/1000.0f - 0.5f);
            sail.set_desired_position(sail_set_pos);
        }
        sail.update(delta_time);

        HALSITL::RCOutput* rcout = dynamic_cast<HALSITL::RCOutput*>(AP_HAL::get_HAL().rcout);
        if (rcout) {
            AP_HAL::ServoStatus status;
            status.homed = AP_HAL::SERVO_HOMED;
            status.moving = fabs(rudder.get_position() - rudder.get_desired_position()) > FLT_EPSILON;
            status.pwm = (-rudder.get_position()*500) + 1500;
            rcout->_actual[STEERING_SERVO_CH] = status;
            status.moving = fabs(mast.get_position() - mast.get_desired_position()) > FLT_EPSILON;
            status.pwm = (mast.get_position()*500) + 1500;
            rcout->_actual[MAST_RAISE_CH] = status;
            status.moving = fabs(sail.get_position() - sail.get_desired_position()) > FLT_EPSILON;
            status.pwm = (sail.get_position()*500) + 1500;
            rcout->_actual[SAIL_ROTATE_CH] = status;
        }
    }

    // calculate apparent wind in earth-frame (this is the direction the wind is coming from)
    // Note than the SITL wind direction is defined as the direction the wind is travelling to
    // This is accounted for in these calculations
    Vector3f wind_apparent_ef = wind_ef + velocity_ef;
    const float wind_apparent_dir_ef = degrees(atan2f(wind_apparent_ef.y, wind_apparent_ef.x));
    const float wind_apparent_speed = safe_sqrt(sq(wind_apparent_ef.x)+sq(wind_apparent_ef.y));

    const float wind_apparent_dir_bf = wrap_180(wind_apparent_dir_ef - degrees(AP::ahrs().yaw));

    // set RPM and airspeed from wind speed, allows to test RPM and Airspeed wind vane back end in SITL
    rpm1 = wind_apparent_speed;
    airspeed_pitot = wind_apparent_speed;

    // calculate angle-of-attack from wind to mainsail
    float aoa_deg = get_angle_of_attack(wind_apparent_dir_bf);

    // calculate Lift force (perpendicular to wind direction) and Drag force (parallel to wind direction)
    float lift_wf, drag_wf;
    calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);

    // rotate lift and drag from wind frame into body frame
    const float sin_rot_rad = sinf(radians(wind_apparent_dir_bf));
    const float cos_rot_rad = cosf(radians(wind_apparent_dir_bf));
    const float force_fwd = (lift_wf * sin_rot_rad - (drag_wf * cos_rot_rad)) * BLUEBOTTLE_FORCE_REDUCTION;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = get_yaw_rate(steering, speed);

    gyro = Vector3f(0,0,radians(yaw_rate)) + wave_gyro;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // hull drag
    float hull_drag = sq(speed) * 0.5f;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // accel in body frame due acceleration from sail and deceleration from hull friction
    accel_body = Vector3f((throttle_force + force_fwd) - hull_drag, 0, 0);
    accel_body /= mass;

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

    // now in earth frame
    // remove roll and pitch effects from waves
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    Matrix3f temp_dcm;
    temp_dcm.from_euler(0.0f, 0.0f, y);
    Vector3f accel_earth = temp_dcm * accel_body;

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0 + wave_heave;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // tide calcs
    Vector3f tide_velocity_ef;
     if (hal.util->get_soft_armed() && !is_zero(sitl->tide.speed) ) {
        tide_velocity_ef.x = -cosf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.y = -sinf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.z = 0.0f;
     }

    // new velocity vector
    velocity_ef_water += accel_earth * delta_time;
    velocity_ef = velocity_ef_water + tide_velocity_ef;

    // new position vector
    position += velocity_ef * delta_time;

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

    // update wave calculations
    update_wave(delta_time);

}

} // namespace SITL
