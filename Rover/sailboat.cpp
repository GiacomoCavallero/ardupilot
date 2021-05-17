#include "Rover.h"

#include <AP_HAL_Linux/RCOutput_Ocius.h>  // send_epos_status()

#define SAILBOAT_AUTO_TACKING_TIMEOUT_MS 15000   // tacks in auto mode timeout if not successfully completed within this many milliseconds
#define SAILBOAT_TACKING_ACCURACY_DEG 10        // tack is considered complete when vehicle is within this many degrees of target tack angle
#define SAILBOAT_NOGO_PAD 10                    // deg, the no go zone is padded by this much when deciding if we should use the Sailboat heading controller
#define TACK_RETRY_TIME_MS 5000                 // Can only try another auto mode tack this many milliseconds after the last is cleared (either competed or timed-out)
/*
To Do List
 - Improve tacking in light winds and bearing away in strong wings
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - max speed parameter and controller, for mapping you may not want to go too fast
 - mavlink sailing messages
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - tack on depth sounder info to stop sailing into shallow water on indirect sailing routes
 - add option to do proper tacks, ie tacking on flat spot in the waves, or only try once at a certain speed, or some better method than just changing the desired heading suddenly
*/

const AP_Param::GroupInfo Sailboat::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Sailboat
    // @Description: This enables Sailboat functionality
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Sailboat, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ANGLE_MIN
    // @DisplayName: Sail min angle
    // @Description: Mainsheet tight, angle between centerline and boom
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MIN", 2, Sailboat, sail_angle_min, 0),

    // @Param: ANGLE_MAX
    // @DisplayName: Sail max angle
    // @Description: Mainsheet loose, angle between centerline and boom. For direct-control rotating masts, the rotation angle at SERVOx_MAX/_MIN; for rotating masts, this value can exceed 90 degrees if the linkages can physically rotate the mast past that angle.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 3, Sailboat, sail_angle_max, 90),

    // @Param: ANGLE_IDEAL
    // @DisplayName: Sail ideal angle
    // @Description: Ideal angle between sail and apparent wind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_IDEAL", 4, Sailboat, sail_angle_ideal, 25),

    // @Param: HEEL_MAX
    // @DisplayName: Sailing maximum heel angle
    // @Description: When in auto sail trim modes the heel will be limited to this value using PID control
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HEEL_MAX", 5, Sailboat, sail_heel_angle_max, 15),

    // @Param: NO_GO_ANGLE
    // @DisplayName: Sailing no go zone angle
    // @Description: The typical closest angle to the wind the vehicle will sail at. the vehicle will sail at this angle when going upwind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NO_GO_ANGLE", 6, Sailboat, sail_no_go, 45),

    // @Param: WNDSPD_MIN
    // @DisplayName: Sailboat minimum wind speed to sail in
    // @Description: Sailboat minimum wind speed to continue sail in, at lower wind speeds the sailboat will motor if one is fitted
    // @Units: knots
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WNDSPD_MIN", 7, Sailboat, sail_windspeed_min, 0),

    // @Param: XTRACK_MAX
    // @DisplayName: Sailing vehicle max cross track error
    // @Description: The sail boat will tack when it reaches this cross track error, defines a corridor of 2 times this value wide, 0 disables
    // @Units: m
    // @Range: 5 25
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("XTRACK_MAX", 8, Sailboat, xtrack_max, 10),

    // @Param: LOIT_RADIUS
    // @DisplayName: Loiter radius
    // @Description: When in sailing modes the vehicle will keep moving within this loiter radius
    // @Units: m
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOIT_RADIUS", 9, Sailboat, loit_radius, 5),

    // @Param: STOW_ERROR
    // @DisplayName: Sailboat error in sail position acceptable when stowing
    // @Description: Sailboat error in sail position acceptable when stowing, outside this range lowering the sail can damage the vessel.
    // @Units: pwm
    // @Range: 0 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STOW_ERROR", 62, Sailboat, sail_stow_error, 10),

    // @Param: MODE
    // @DisplayName: Sailboat sailing mode
    // @Description: Sailboat sailing mode: 0: MOTOR_ONLY, 1: MOTOR_SAIL, 2: SAIL_ONLY, 3: WAVE_POWER, 4, MOTOR_SOLAR
    // @Units: enum
    // @Range: 0 4
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MODE", 61, Sailboat, sail_mode, 0),

    // @Param: WNDSPD_MAX
    // @DisplayName: Sailboat maximum wind speed to sail in
    // @Description: Sailboat maximum wind speed to continue sail in, at lower wind speeds the sailboat will motor if one is fitted
    // @Units: knots
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WNDSPD_MAX", 60, Sailboat, sail_windspeed_max, 15),

    // @Param: FLAGS
    // @DisplayName: Sailboat bit flags to activate/deactivate skills
//     @Description: Sailboat bit flags (1: Jibe only, 2: Ignore X-Track, 4: Always Set heading to WP)
    // @Units: bits
    // @Range: 0 16
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLAGS", 59, Sailboat, sail_flags, 0),

    // @Param: EPOS_ZERO
    // @DisplayName: Sail encoder 0 point
//     @Description: Encoder position of sail at PWM 1500
    // @Units: ticks
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("EPOS_ZERO", 58, Sailboat, sail_epos_zero, 0),

    // @Param: HOLD_MODE
    // @DisplayName: Sail hold mode
//     @Description: Sail hold mode, 0: drift return, 1 active station hold, 2 figure 8
    // @Units: enumeration
    // @Range: 0..2
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HOLD_MODE", 57, Sailboat, hold_mode, 0),

    // @Param: HOLD_RADIUS
    // @DisplayName: Sail hold radius
//     @Description: Radius around the hold waypoint within we wish to remain
    // @Units: metres
    // @Range: >=0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HOLD_RADIUS", 56, Sailboat, hold_radius, 0),

    // @Param: MTIME_UP
    // @DisplayName: Mast time up
//     @Description: Time to run hydraulic pump to raise mast.
    // @Units: milliseconds
    // @Range: >0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MTIME_UP", 55, Sailboat, mast_time_up, 7500),

    // @Param: MTIME_DOWN
    // @DisplayName: Mast time down
//     @Description: Time to run hydraulic pump to lower mast.
    // @Units: milliseconds
    // @Range: >0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MTIME_DOWN", 54, Sailboat, mast_time_down, 7500),

    // @Param: MTIME_DELAY
    // @DisplayName: Mast time delay
//     @Description: Delay at start and end of MTIME when the motor speed is 0, to allow the relays to set.
    // @Units: milliseconds
    // @Range: >0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MTIME_DELAY", 53, Sailboat, mast_time_delay, 0),

    // @Param: ANGLE_ERR
    // @DisplayName: Sail angle error
    // @Description: Required change in sail angle before moving the set point
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_ERR", 52, Sailboat, sail_angle_error, 5),

    // @Param: TACK_CORRID
    // @DisplayName: Tack Corridor
    // @Description: Maximum acceptable crosstrack error while tacking upwind
    // @Units: m
    // @Range: 0+
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TACK_CORRID", 51, Sailboat, sail_tack_corridor, 100),

    // @Param: TILT_IMU
    // @DisplayName: ID of Mast Tilt IMU
    // @Description: Component ID of the IMU on the mast
    // @Units: CompID
    // @Range: 0+
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TILT_IMU", 20, Sailboat, tilt_imu, 0),

    // @Param: TILT_ERR
    // @DisplayName: Tilt IMU Error
    // @Description: Accepted error in position of the mast up/down
    // @Units: PWM
    // @Range: 0+
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TILT_ERR", 21, Sailboat, tilt_err, 50),

    // @Param: TILT_FILT
    // @DisplayName: Tilt IMU Filter Length
    // @Description: Time over which the measured mast tilt is measured
    // @Units: Seconds
    // @Range: 0+
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TILT_FILT", 22, Sailboat, tilt_filt, 2.0),

    AP_GROUPEND
};

/*
  constructor
 */
Sailboat::Sailboat()
{
    stowing_sail = false;
    AP_Param::setup_object_defaults(this, var_info);
}

bool Sailboat::sail_enabled() const {
    if (!enable)
        return false;

    enum frame_class frame = (enum frame_class)rover.g2.frame_class.get();
    if (frame == FRAME_BLUEBOTTLE) {
        return sail_is_safe();
    } else if (frame == FRAME_WAMV) {
        return false;
    }
    return true;
}

// true if sailboat navigation (aka tacking) is enabled
bool Sailboat::tack_enabled() const
{
    // tacking disabled if not a sailboat
    if (!sail_enabled()) {
        return false;
    }

    // tacking disabled if motor is always on
    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) {
        return false;
    }

    if (rover.g2.frame_class == FRAME_BLUEBOTTLE && !(sail_mode == SAIL_ONLY || sail_mode == MOTOR_SAIL)) {
        return false;
    }

    // disable tacking if motor is available and wind is below cutoff
    if (motor_assist_low_wind()) {
        return false;
    }

    // otherwise tacking is enabled
    return true;
}

void Sailboat::init()
{
    // sailboat defaults
    if (sail_enabled()) {
        rover.g2.crash_angle.set_default(0);
    }

    if (tack_enabled()) {
        rover.g2.loit_type.set_default(1);
    }

    // initialise motor state to USE_MOTOR_ASSIST
    // this will silently fail if there is no motor attached
    set_motor_state(UseMotor::USE_MOTOR_ASSIST, false);
}

// initialise rc input (channel_mainsail), may be called intermittently
void Sailboat::init_rc_in()
{
    // get auxiliary throttle value
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MAINSAIL);
    if (rc_ptr != nullptr) {
        // use aux as sail input if defined
        channel_mainsail = rc_ptr;
        channel_mainsail->set_angle(100);
        channel_mainsail->set_default_dead_zone(30);
    } else {
        // use throttle channel
        channel_mainsail = rover.channel_throttle;
    }
}

// decode pilot mainsail input and return in steer_out and throttle_out arguments
// mainsail_out is in the range 0 to 100, defaults to 100 (fully relaxed) if no input configured
void Sailboat::get_pilot_desired_mainsail(float &mainsail_out, float &wingsail_out, float &mast_rotation_out)
{
    // no RC input means mainsail is moved to trim
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (channel_mainsail == nullptr)) {
        mainsail_out = 100.0f;
        wingsail_out = 0.0f;
        mast_rotation_out = 0.0f;
        return;
    }
    mainsail_out = constrain_float(channel_mainsail->get_control_in(), 0.0f, 100.0f);
    wingsail_out = constrain_float(channel_mainsail->get_control_in(), -100.0f, 100.0f);
    mast_rotation_out = constrain_float(channel_mainsail->get_control_in(), -100.0f, 100.0f);
}

// calculate throttle and mainsail angle required to attain desired speed (in m/s)
// returns true if successful, false if sailboats not enabled
void Sailboat::get_throttle_and_mainsail_out(float desired_speed, float &throttle_out, float &mainsail_out, float &wingsail_out, float &mast_rotation_out)
{
    if (!sail_enabled()) {
        throttle_out = 0.0f;
        mainsail_out = 0.0f;
        wingsail_out = 0.0f;
        mast_rotation_out = 0.0f;
        return;
    }

    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        // TODO: adjust sail angle
        if (sail_is_safe()) {
            if (sail_mode == SAIL_ONLY || sail_mode == MOTOR_SAIL || sail_mode == MOTOR_SOLAR) {
                int32_t sail_set_pos = AP_HAL::get_HAL().rcout->read(SAIL_SERVO_CH-1);
                int32_t optimal_pos = get_optimal_sail_position();

                if (optimal_pos != 0 && abs(optimal_pos - sail_set_pos) >= sail_stow_error) {
                    set_sail_position(optimal_pos);
                }
            }
        }

        bool tack_push_required = false;
        if (sail_mode == SAIL_ONLY) {
            // do we need push through a tack?
            if (tacking()) {
                tack_push_required = true;
            } else {
                // If not tacking, but nose gets too high, need to use throttle to help push nose down
                // FIXME: this should use the water/true wind
                float relative_wind = wrap_PI(rover.g2.windvane.get_true_wind_direction_rad() - AP::ahrs().get_yaw());
                relative_wind = fabs(relative_wind);
                float sail_no_go_rad = radians(sail_no_go);
                if (relative_wind < sail_no_go_rad) {
                    // Nose is higher than no_go
                    tack_push_required = true;
                    if (relative_wind > (sail_no_go_rad / 2)) {
                        // nose is just a bit high, so we reduce the desired speed for the throttle
                        desired_speed = desired_speed * MAX((sail_no_go_rad - relative_wind) * 2, 0);
//                        printf("reducing desired speed - rel wind:%.1f, speed: %.2f\n",
//                                degrees(relative_wind), desired_speed);
                    }
                }
            }
        }

        // Throttle up in MOTOR_SAIL while tacking.
        if (sail_mode == MOTOR_ONLY || sail_mode == MOTOR_SAIL ||
                sail_mode == MOTOR_SOLAR ||
                (sail_mode == SAIL_ONLY && tack_push_required) ||
                (sail_mode == WAVE_POWER &&
                        desired_speed > KNOTS_PER_METRE * (0.5*1.1)))
        {
            throttle_out = 100.0f * rover.g2.attitude_control.get_throttle_out_speed(desired_speed,
                                                                            rover.g2.motors.limit.throttle_lower,
                                                                            rover.g2.motors.limit.throttle_upper,
                                                                            rover.g.speed_cruise,
                                                                            rover.g.throttle_cruise * 0.01f,
                                                                            rover.G_Dt);
        } else {
            throttle_out = 0.0f;
        }

        if (throttle_out < 0.0f) {
            // Never want negative throttle in auto modes.
            throttle_out = 0.0f;
        }

        mainsail_out = 100.0f;
        return;
    }

    // run speed controller if motor is forced on or motor assistance is required for low speeds or tacking
    if ((motor_state == UseMotor::USE_MOTOR_ALWAYS) ||
         motor_assist_tack() ||
         motor_assist_low_wind()) {
        // run speed controller - duplicate of calls found in mode::calc_throttle();
        throttle_out = 100.0f * rover.g2.attitude_control.get_throttle_out_speed(desired_speed,
                                                                        rover.g2.motors.limit.throttle_lower,
                                                                        rover.g2.motors.limit.throttle_upper,
                                                                        rover.g.speed_cruise,
                                                                        rover.g.throttle_cruise * 0.01f,
                                                                        rover.G_Dt);
    } else {
        throttle_out = 0.0f;
    }

    // if we are motoring relax sails
    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) {
        mainsail_out = 100.0f;
        wingsail_out = 0.0f;
        mast_rotation_out = 0.0f;
        return;
    }

    // use PID controller to sheet out, this number is expected approximately in the 0 to 100 range (with default PIDs)
    const float pid_offset = rover.g2.attitude_control.get_sail_out_from_heel(radians(sail_heel_angle_max), rover.G_Dt) * 100.0f;

    // get apparent wind, + is wind over starboard side, - is wind over port side
    const float wind_dir_apparent = degrees(rover.g2.windvane.get_apparent_wind_angle_rad());
    const float wind_dir_apparent_abs = fabsf(wind_dir_apparent);
    const float wind_dir_apparent_sign = is_negative(wind_dir_apparent) ? -1.0f : 1.0f;

    //
    // mainsail control
    //

    // main sails cannot be used to reverse
    if (!is_positive(desired_speed)) {
        mainsail_out = 100.0f;
    } else {
        // Sails are sheeted the same on each side use abs wind direction

        // set the main sail to the ideal angle to the wind
        float mainsail_angle = wind_dir_apparent_abs - sail_angle_ideal;

        // make sure between allowable range
        mainsail_angle = constrain_float(mainsail_angle,sail_angle_min, sail_angle_max);

        // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
        float mainsail_base = linear_interpolate(0.0f, 100.0f, mainsail_angle,sail_angle_min,sail_angle_max);

        mainsail_out = constrain_float((mainsail_base + pid_offset), 0.0f ,100.0f);
    }

    //
    // wingsail control
    //

    // wing sails auto trim, we only need to reduce power if we are tipping over, must also be trimmed for correct tack
    // dont allow to reduce power to less than 0, ie not backwinding the sail to self-right
    wingsail_out = (100.0f - MIN(pid_offset,100.0f)) * wind_dir_apparent_sign;

    // wing sails can be used to go backwards, probably not recommended though
    if (is_negative(desired_speed)) {
        wingsail_out *= -1.0f;
    }

    //
    // direct mast rotation control
    //

    if (!is_positive(desired_speed)) {
        // rotating sails can be used to reverse, but not in this version
        mast_rotation_out = 0.0f;
    } else {

        if (wind_dir_apparent_abs < sail_angle_ideal) {
            // in irons, center the sail.
            mast_rotation_out = 0.0f;

        } else {

            float mast_rotation_angle;
            if (wind_dir_apparent_abs < (90.0f + sail_angle_ideal)) {
                // use sail as a lift device, at ideal angle of attack, but depower to prevent excessive heel
                // multiply pid_offset by 0.01 to keep the scaling in the same range as the other sail outputs
                // this means the default PIDs should apply reasonably well to all sail types
                mast_rotation_angle = wind_dir_apparent_abs - sail_angle_ideal * MAX(1.0f - pid_offset*0.01f,0.0f);

                // restore sign
                mast_rotation_angle *= wind_dir_apparent_sign;

            } else {
                // use sail as drag device, but avoid wagging the sail as the wind oscillates
                // between 180 and -180 degrees
                mast_rotation_angle = 90.0f;
                if (wind_dir_apparent_abs > 135.0f) {
                    // wind is almost directly behind, keep wing on current tack
                    if (SRV_Channels::get_output_scaled(SRV_Channel::k_mast_rotation) < 0) {
                        mast_rotation_angle *= -1.0f;
                    }
                } else {
                    // set the wing on the correct tack, so that is can be sheeted in if required
                    mast_rotation_angle *= wind_dir_apparent_sign;
                }
            }

            // linear interpolate servo displacement (-100 to 100) from mast rotation angle and restore sign
            mast_rotation_out = linear_interpolate(-100.0f, 100.0f, mast_rotation_angle, -sail_angle_max, sail_angle_max);
        }
    }

}

// Velocity Made Good, this is the speed we are traveling towards the desired destination
// only for logging at this stage
// https://en.wikipedia.org/wiki/Velocity_made_good
float Sailboat::get_VMG() const
{
    // return zero if we don't have a valid speed
    float speed;
    if (!rover.g2.attitude_control.get_forward_speed(speed)) {
        return 0.0f;
    }

    // return speed if not heading towards a waypoint
    if (!rover.control_mode->is_autopilot_mode()) {
        return speed;
    }

    return (speed * cosf(wrap_PI(radians(rover.g2.wp_nav.wp_bearing_cd() * 0.01f) - rover.ahrs.yaw)));
}

// handle user initiated tack while in acro mode
void Sailboat::handle_tack_request_acro()
{
    if (!tack_enabled() || currently_tacking) {
        return;
    }
    // set tacking heading target to the current angle relative to the true wind but on the new tack
    currently_tacking = true;
    // FIXME: this should use the water/true wind
    tack_heading_rad = wrap_2PI(rover.ahrs.yaw + 2.0f * wrap_PI((rover.g2.windvane.get_true_wind_direction_rad() - rover.ahrs.yaw)));

    tack_request_ms = AP_HAL::millis();
}

// return target heading in radians when tacking (only used in acro)
float Sailboat::get_tack_heading_rad()
{
    if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.yaw)) < radians(SAILBOAT_TACKING_ACCURACY_DEG) ||
       ((AP_HAL::millis() - tack_request_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
        clear_tack();
    }

    return tack_heading_rad;
}

// handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
void Sailboat::handle_tack_request_auto()
{
    if (!tack_enabled() || currently_tacking) {
        return;
    }

    // record time of request for tack.  This will be processed asynchronously by sailboat_calc_heading
    tack_request_ms = AP_HAL::millis();
}

// clear tacking state variables
void Sailboat::clear_tack()
{
    currently_tacking = false;
    tack_assist = false;
    tack_request_ms = 0;
    tack_clear_ms = AP_HAL::millis();
}

// returns true if boat is currently tacking
bool Sailboat::tacking() const
{
    return tack_enabled() && currently_tacking;
}

// returns true if sailboat should take a indirect navigation route to go upwind
// desired_heading should be in centi-degrees
bool Sailboat::use_indirect_route(float desired_heading_cd) const
{
    if (!tack_enabled()) {
        return false;
    }

    // use sailboat controller until tack is completed
    if (currently_tacking) {
        return true;
    }

    // convert desired heading to radians
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // check if desired heading is in the no go zone, if it is we can't go direct
    // pad no go zone, this allows use of heading controller rather than L1 when close to the wind
    // FIXME: this should use the water/true wind
    return fabsf(wrap_PI(rover.g2.windvane.get_true_wind_direction_rad() - desired_heading_rad)) <= radians(sail_no_go + SAILBOAT_NOGO_PAD);
}

// if we can't sail on the desired heading then we should pick the best heading that we can sail on
// this function assumes the caller has already checked sailboat_use_indirect_route(desired_heading_cd) returned true
float Sailboat::calc_heading(float desired_heading_cd)
{
    if (!tack_enabled()) {
        return desired_heading_cd;
    }
    bool should_tack = false;

    // find which tack we are on
    const AP_WindVane::Sailboat_Tack current_tack = rover.g2.windvane.get_current_tack();

    // convert desired heading to radians
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // if the desired heading is outside the no go zone there is no need to change it
    // this allows use of heading controller rather than L1 when desired
    // this is used in the 'SAILBOAT_NOGO_PAD' region
    // FIXME: this should use the water/true wind
    const float true_wind_rad = rover.g2.windvane.get_true_wind_direction_rad();
    if (fabsf(wrap_PI(true_wind_rad - desired_heading_rad)) > radians(sail_no_go) && !currently_tacking) {

        // calculate the tack the new heading would be on
        const float new_heading_apparent_angle = wrap_PI(true_wind_rad - desired_heading_rad);
        AP_WindVane::Sailboat_Tack new_tack;
        if (is_negative(new_heading_apparent_angle)) {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_PORT;
        } else {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_STARBOARD;
        }

        // if the new tack is not the same as the current tack we need might need to tack
        if (new_tack != current_tack) {
            // see if it would be a tack, the front of the boat going through the wind
            // or a gybe, the back of the boat going through the wind
            const float app_wind_rad = rover.g2.windvane.get_apparent_wind_angle_rad();
            if (fabsf(app_wind_rad) + fabsf(new_heading_apparent_angle) < M_PI) {
                should_tack = true;
            }
        }

        if (!should_tack) {
            return desired_heading_cd;
        }
    }

    // check for user requested tack
    uint32_t now = AP_HAL::millis();
    if (tack_request_ms != 0 && !should_tack  && !currently_tacking) {
        // set should_tack flag is user requested tack within last 0.5 sec
        should_tack = ((now - tack_request_ms) < 500);
        tack_request_ms = 0;
    }

    // trigger tack if cross track error larger than xtrack_max parameter
    // this effectively defines a 'corridor' of width 2*xtrack_max that the boat will stay within
    const float cross_track_error = rover.g2.wp_nav.crosstrack_error();
    if ((fabsf(cross_track_error) >= sail_tack_corridor) && !is_zero(sail_tack_corridor) && !should_tack && !currently_tacking) {
        // make sure the new tack will reduce the cross track error
        // if were on starboard tack we are traveling towards the left hand boundary
        if (is_positive(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_STARBOARD)) {
            should_tack = true;
        }
        // if were on port tack we are traveling towards the right hand boundary
        if (is_negative(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT)) {
            should_tack = true;
        }
    }

    // calculate left and right no go headings looking upwind, Port tack heading is left no-go, STBD tack is right of no-go
    const float left_no_go_heading_rad = wrap_2PI(true_wind_rad + radians(sail_no_go));
    const float right_no_go_heading_rad = wrap_2PI(true_wind_rad - radians(sail_no_go));

    // if tack triggered, calculate target heading
    if (should_tack && (now - tack_clear_ms) > TACK_RETRY_TIME_MS) {
        gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking");
        // calculate target heading for the new tack
        switch (current_tack) {
            case AP_WindVane::Sailboat_Tack::TACK_PORT:
                tack_heading_rad = right_no_go_heading_rad;
                break;
            case AP_WindVane::Sailboat_Tack::TACK_STARBOARD:
                tack_heading_rad = left_no_go_heading_rad;
                break;
        }
        currently_tacking = true;
        auto_tack_start_ms = now;
    }

    // if we are tacking we maintain the target heading until the tack completes or times out
    if (currently_tacking) {
        // check if we have reached target
        if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.yaw)) <= radians(SAILBOAT_TACKING_ACCURACY_DEG)) {
            clear_tack();
        } else if ((now - auto_tack_start_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS) {
            // tack has taken too long
            if ((motor_state == UseMotor::USE_MOTOR_ASSIST) && (now - auto_tack_start_ms) < (3.0f * SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
                // if we have throttle available use it for another two time periods to get the tack done
                tack_assist = true;
            } else if ((now - auto_tack_start_ms) < (3.0f * SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
                // Assisted or not, tacks should have same timeout length
                gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking timed out");
                clear_tack();
            }
        }
        // return tack target heading
        return degrees(tack_heading_rad) * 100.0f;
    }

    // return the correct heading for our current tack
    if (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT) {
        return degrees(left_no_go_heading_rad) * 100.0f;
    } else {
        return degrees(right_no_go_heading_rad) * 100.0f;
    }
}

// set state of motor
void Sailboat::set_motor_state(UseMotor state, bool report_failure)
{
    // always allow motor to be disabled
    if (state == UseMotor::USE_MOTOR_NEVER) {
        motor_state = state;
        return;
    }

    // enable assistance or always on if a motor is defined
    if (rover.g2.motors.have_skid_steering() ||
        SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
        rover.get_frame_type() != rover.g2.motors.frame_type::FRAME_TYPE_UNDEFINED) {
        motor_state = state;
    } else if (report_failure) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Sailboat: failed to enable motor");
    }
}

// set the sailing mode
void Sailboat::set_sail_mode(SailMode sailmode) {
    switch (sailmode) {
    case MOTOR_ONLY: case MOTOR_SOLAR:
        set_motor_state(UseMotor::USE_MOTOR_ALWAYS);
        break;

    case MOTOR_SAIL: case WAVE_POWER:
        set_motor_state(UseMotor::USE_MOTOR_ASSIST);
        break;

    case SAIL_ONLY:
        set_motor_state(UseMotor::USE_MOTOR_NEVER);
    }
}

// true if motor is on to assist with slow tack
bool Sailboat::motor_assist_tack() const
{
    // throttle is assist is disabled
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) {
        return false;
    }

    // assist with a tack because it is taking too long
    return tack_assist;
}

// true if motor should be on to assist with low wind
bool Sailboat::motor_assist_low_wind() const
{
    // motor assist is disabled
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) {
        return false;
    }

    // assist if wind speed is below cutoff
    return (is_positive(sail_windspeed_min) &&
            rover.g2.windvane.wind_speed_enabled() &&
            wind_strength == WIND_LOW);
}

MAV_RESULT Sailboat::set_servo(uint8_t channel, uint16_t pwm, bool gcs_command) {
    AP_ServoRelayEvents *handler = AP::servorelayevents();
    if (handler == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        if (channel == MAST_SERVO_CH) {
            return set_mast_position(pwm, gcs_command);
        } else if (channel == SAIL_SERVO_CH) {
            return set_sail_position(pwm, gcs_command);
        } else if (channel == WINCH_SERVO_CH) {
            return set_winch_position(pwm, gcs_command);
        }
    }
    if (handler->do_set_servo(channel, pwm)) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

MAV_RESULT Sailboat::set_mast_position(uint16_t pwm, bool gcs_command) {
    uint16_t mast_set_pos = AP_HAL::get_HAL().rcout->read(MAST_SERVO_CH-1);
    uint16_t sail_set_pos = AP_HAL::get_HAL().rcout->read(SAIL_SERVO_CH-1);
    AP_HAL::ServoStatus mast_status = AP_HAL::get_HAL().rcout->read_status(MAST_SERVO_CH-1);
    AP_HAL::ServoStatus sail_status = AP_HAL::get_HAL().rcout->read_status(SAIL_SERVO_CH-1);
    int32_t diff = mast_set_pos;
    diff = abs(diff - (int32_t)pwm);
    if (mast_set_pos == pwm) {
        if (mast_status.moving || diff < 100) {
            // If we're already in position or still moving to the position no need to do anything.
            return MAV_RESULT_ACCEPTED;
        }
    }

    if (!rover.arming.is_armed() && !gcs_command) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    if (pwm > 1800) {
    } else if (sail_status.homed == AP_HAL::SERVO_HOMED &&
            !sail_status.moving &&
            (abs(sail_set_pos - 1500) <= 10 || sail_set_pos == 0) &&
            abs(sail_status.pwm - 1500) <= sail_stow_error) {
        // Sail is homed, not moving and within limits of center, safe to lower
    } else {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    AP_ServoRelayEvents *handler = AP::servorelayevents();
    if (handler->do_set_servo(MAST_SERVO_CH, pwm)) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

MAV_RESULT Sailboat::set_sail_position(uint16_t pwm, bool gcs_command) {
    uint16_t sail_set_pos = AP_HAL::get_HAL().rcout->read(SAIL_SERVO_CH-1);
    AP_HAL::ServoStatus sail_status = AP_HAL::get_HAL().rcout->read_status(SAIL_SERVO_CH-1);
    int32_t diff = sail_set_pos;
    diff = abs(diff - (int32_t)pwm);

    int32_t pwm_err = sail_angle_error * 400 / 90;  // Get the PWM error given the allowed angle error
    if (!gcs_command && diff <= pwm_err) {
        // New position is within error of the current set point

        // How far is the current position from the desired position
        int32_t pwm_dist = sail_status.pwm;
        pwm_dist = abs(pwm_dist - (int32_t)pwm);

        if (sail_status.moving || pwm_dist <= pwm_err) {
            // Sail is still moving to the set point or within error of the desired point
            return MAV_RESULT_ACCEPTED;
        }
    }

    // Only move if armed or received a GCS command to center the sail.
    if (gcs_command && pwm == 1500) {
    } else if (!rover.arming.is_armed()) {
        if (gcs_command) {
            gcs().send_text(MAV_SEVERITY_WARNING, "GCS Command Reject - Not Armed");
        }
        return MAV_RESULT_TEMPORARILY_REJECTED;
    } else if (pwm != 1500 && !sail_is_safe(gcs_command)) {
        // Sail is not safe to move.
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    AP_ServoRelayEvents *handler = AP::servorelayevents();
    if (handler->do_set_servo(SAIL_SERVO_CH, pwm)) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

MAV_RESULT Sailboat::set_winch_position(uint16_t pwm, bool gcs_command) {
    // TODO: Sailboat::set_winch_position
    if (!rover.arming.is_armed()) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    } else if (pwm >= 1 && pwm <= 500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Bluebottle: Winch emergency stop received.");
    }

    AP_ServoRelayEvents *handler = AP::servorelayevents();
    if (handler->do_set_servo(WINCH_SERVO_CH, pwm)) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

bool Sailboat::sail_is_safe(bool gcs_command) const {
    AP_HAL::ServoStatus mast_status = AP_HAL::get_HAL().rcout->read_status(MAST_SERVO_CH-1);
    AP_HAL::ServoStatus sail_status = AP_HAL::get_HAL().rcout->read_status(SAIL_SERVO_CH-1);
    uint16_t mast_set_pos = AP_HAL::get_HAL().rcout->read(MAST_SERVO_CH-1);

    //Check mast is homed
    if (tilt_imu != 0 && mast_status.pwm != 0) {
        // Tilt sensor is enabled and we have a position, we assume we're homed at that position
    } else if (mast_status.homed != AP_HAL::SERVO_HOMED) {
        if (gcs_command) {
            gcs().send_text(MAV_SEVERITY_WARNING, "GCS Command Reject - Mast not homed");
        }
        return false;
    }

    // Check sail is homed
    if (sail_status.homed != AP_HAL::SERVO_HOMED) {
        if (gcs_command) {
            gcs().send_text(MAV_SEVERITY_WARNING, "GCS Command Reject - Sail not homed");
        }
        return false;
    }

    // Check mast is not moving
    if (mast_status.moving) {
        if (gcs_command) {
            gcs().send_text(MAV_SEVERITY_WARNING, "GCS Command Reject - Mast moving");
        }
        return false;
    }

    // Check mast is upright
    if (mast_status.pwm < (1900 - tilt_err)) {
        if (gcs_command) {
            gcs().send_text(MAV_SEVERITY_WARNING, "GCS Command Reject - Mast not fully raised");
        }
        return false;
    }

    // Check mast has not been told to lower
    if (tilt_imu != 0 && mast_set_pos == 0) {
        // If set_pos is 0, skip if tilt_imu is enabled
    } else if (mast_set_pos < (1900 - 10)) {
        if (gcs_command) {
            gcs().send_text(MAV_SEVERITY_WARNING, "GCS Command Reject - Mast told to lower");
        }
        return false;
    }

    // Check sail isn't currently stowing
    if (stowing_sail) {
        if (gcs_command) {
            gcs().send_text(MAV_SEVERITY_WARNING, "GCS Command Reject - Sail is stowing");
        }
        return false;
    }

    return true;
}

static float linInterp(const double x1, const double x2, const double y1, const double y2, const double x)
{
    return y1 + (x - x1) / (x2 - x1) * (y2 - y1);
}

extern float calcSun(float lat, float lon);
uint16_t Sailboat::get_optimal_sail_position() const {
    uint16_t pwm = 0;
    if (sail_mode == MOTOR_SOLAR) {
        float azi = calcSun((float)rover.current_loc.lat, (float)rover.current_loc.lng);
        if (azi > -1) {
//            float bearing = gps.bearing_true() / 100.0;
            float bearing = ToDeg(rover.ahrs.yaw);
            float sun_angle = wrap_360(azi - bearing);

            uint16_t sail_set_pos = AP_HAL::get_HAL().rcout->read(SAIL_SERVO_CH-1);
            int32_t pwm_err = sail_angle_error * 400 / 90;  // Get the PWM error given the allowed angle error

            pwm = 1500;
            if (sun_angle >= 90 - sail_angle_error && sun_angle <= 90 + sail_angle_error) {
                // sun_angle is about 90 degrees, need to add hysteresis
                if (sun_angle <= 90 && sail_set_pos < (1100 + pwm_err)) {
                    sun_angle = 90.5;
                } else if (sun_angle >= 90 && sail_set_pos > (1900 - pwm_err)) {
                    sun_angle = 89.5;
                }
            } else if (sun_angle >= 270 - sail_angle_error && sun_angle <= 270 + sail_angle_error) {
                // sun_angle is about 270 degrees, need to add hysteresis
                if (sun_angle <= 270 && sail_set_pos < (1100 + pwm_err)) {
                    sun_angle = 270.5;
                } else if (sun_angle >= 270 && sail_set_pos > (1900 - pwm_err)) {
                    sun_angle = 269.5;
                }
            }


            if (sun_angle <= 90) {
                pwm = 1500 + ((sun_angle)/90) * 400;
            }
            if (sun_angle > 90 && sun_angle <= 180) {
                pwm = 1500 - ((180-sun_angle)/90) * 400;
            }
            if (sun_angle > 180 && sun_angle <= 270) {
                pwm = 1500 + ((sun_angle-180)/90) * 400;
            }
            if (sun_angle > 270) {
                pwm = 1500 - ((360-sun_angle)/90) * 400;
            }
        }
    } else if (sail_mode == MOTOR_SAIL || sail_mode == SAIL_ONLY) {
        float desired_angle = 0;

        const float apparent_wind_ang = ToDeg(rover.g2.windvane.get_apparent_wind_angle_rad());
        const float apparent_wind_speed = rover.g2.windvane.get_apparent_wind_speed();

        const float target_trim_angle = std::max(sail_angle_ideal - 0.4*(apparent_wind_speed*apparent_wind_speed), 0.0);

        const float trim_start = 90 + target_trim_angle;

        const float upwind_target = 35;
        const float upwind_trim = 16;

        float desired_trim = 0;

        if (fabs(apparent_wind_ang) >= trim_start) {
            desired_trim = 90;
            desired_angle = 0;
        } else if (fabs(apparent_wind_ang) < upwind_target) {
            // Finer than target - interpolate between upwind trim and 0
            desired_trim = linInterp(0, upwind_target, 0, upwind_trim, fabs(apparent_wind_ang));
            desired_angle = std::copysign(90 - desired_trim, apparent_wind_ang);
        } else {
            // Trimmable zone - interpolate between 90 and upwind trim
            desired_trim = linInterp(upwind_target, trim_start, upwind_trim, 90, fabs(apparent_wind_ang));
            desired_angle = std::copysign(90 - desired_trim, apparent_wind_ang);
        }

        pwm =  1500 - (desired_angle * 400) / 90;
    }

    // Restrict the possible pwm values to 1100--1900, this should be unnecessary, but just in case
    if (pwm < 1100)
        pwm = 1100;
    else if (pwm > 1900)
        pwm = 1900;

    return pwm;
}

void Sailboat::check_wind() {
    static uint32_t extremeWeatherStart = 0;
    static uint32_t timeWeatherCleared = 0;
    static uint32_t calmWeatherStart = 0;
    static uint32_t calmWeatherEnd = 0;

    // TODO: determine wind strength
    if(!rover.g2.windvane.wind_speed_enabled()) {
        wind_strength = WIND_UNKNOWN;
        return;
    }

    float wind_speed = rover.g2.windvane.get_true_wind_speed();
    float max_wind = sail_windspeed_max / KNOTS_PER_METRE;
    float min_wind = sail_windspeed_min / KNOTS_PER_METRE;

    if (wind_speed > max_wind) {
        if (extremeWeatherStart == 0) {
            extremeWeatherStart = millis();
        }
        timeWeatherCleared = 0;

        if (wind_strength != WIND_HIGH) {
            if (wind_speed >= (1.2 * max_wind)) {
                // If wind gusts are 20% above the max, wind is high
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Strong wind gusts. (%.1f kts)", wind_speed * KNOTS_PER_METRE);
                wind_strength = WIND_HIGH;
            } else if (millis() - extremeWeatherStart >= 30000) {
                // If sustained wind is above the max for 30 seconds, it is high
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Sustained high winds. (%.1f kts)", wind_speed * KNOTS_PER_METRE);
                wind_strength = WIND_HIGH;
            }
        }
    } else if (wind_strength == WIND_HIGH && wind_speed < (0.8 * max_wind)) {
        if (timeWeatherCleared == 0) {
            timeWeatherCleared = millis();
        }
        extremeWeatherStart = 0;
    } else {
        timeWeatherCleared = 0;
        extremeWeatherStart = 0;
    }

    if (timeWeatherCleared != 0 && millis() - timeWeatherCleared > 30000) {
        // Weather has cleared, raise sail if in a sailing mode
        gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Extreme weather cleared.");
        wind_strength = WIND_STRONG;
        calmWeatherStart = calmWeatherEnd = 0;
    } else if (wind_speed < min_wind) {
        calmWeatherEnd = 0;
        if (calmWeatherStart == 0) {
            calmWeatherStart = millis();
        } else if ((millis() - calmWeatherStart) > 10000) {
            // If wind is below the min for 10 seconds, it is low.
            if (wind_strength != WIND_LOW && sail_enabled() &&
                    (sail_mode == MOTOR_SAIL || sail_mode == SAIL_ONLY)) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Wind has dropped, too low for sailing.");
            }
            wind_strength = WIND_LOW;
        }
    } else if (wind_strength != WIND_HIGH){
        calmWeatherStart = 0;
        if (wind_strength == WIND_LOW) {
            if (calmWeatherEnd == 0) {
                calmWeatherEnd = millis();
            } else if ((millis() - calmWeatherEnd) > 10000) {
                // If wind is above min for 10 seconds, it is fair wind for sailing
                wind_strength = WIND_FAIR;
                if (sail_enabled() && (sail_mode == MOTOR_SAIL || sail_mode == SAIL_ONLY)) {
                    gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Wind has risen, sailing can resume.");
                }
            }
        } else if (wind_speed < ((min_wind + max_wind) / 2.0)) {
            wind_strength = WIND_FAIR;
        } else {
            wind_strength = WIND_STRONG;
        }
    }
}

static bool compass_is_enabled = true;

void Sailboat::sail_guard() {
    // determine wind strength
    check_wind();

    if (AP::compass().enabled() && !compass_is_enabled) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Sailboat: (sail_guard) Compass has been disabled.");
        hal.console->printf("Sailboat: (sail_guard) Compass has been disabled.\n");
        compass_is_enabled = true;
    } else if (!AP::compass().enabled() && compass_is_enabled) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Sailboat: (sail_guard) Compass has been enabled.");
        hal.console->printf("Sailboat: (sail_guard) Compass has been enabled.\n");
        compass_is_enabled = false;
    }

    if (!rover.arming.is_armed() || !enable) {
        // Cannot automatically home/stow/raise sail/mast if disarmed or sail not enabled in params.
        return;
    }

    if (rover.g2.frame_class != FRAME_BLUEBOTTLE) {
        // Vehicle not designed to stow sail.
        return;
    }

    static uint32_t last_not_home_msg = 0;
    uint16_t mast_set_pos = AP_HAL::get_HAL().rcout->read(MAST_SERVO_CH-1);
    AP_HAL::ServoStatus mast_status = AP_HAL::get_HAL().rcout->read_status(MAST_SERVO_CH-1);
    AP_HAL::ServoStatus sail_status = AP_HAL::get_HAL().rcout->read_status(SAIL_SERVO_CH-1);
    if (sail_status.homed == AP_HAL::SERVO_HOMED && tilt_imu != 0 && mast_status.pwm != 0) {
        // The sail is homed, and we assume the mast is at the current/last PWM from the tilt sensor
    } else if (mast_status.homed != AP_HAL::SERVO_HOMED || sail_status.homed != AP_HAL::SERVO_HOMED) {
        // Mast or Sail not homed
        uint32_t now = millis();
        if (last_not_home_msg == 0 || (now - last_not_home_msg) > 5000) {
            if (mast_status.homed != AP_HAL::SERVO_HOMED) {
                printf("Sailboat::sail_guard() - Mast not yet homed.\n");
            }
            if (sail_status.homed != AP_HAL::SERVO_HOMED) {
                printf("Sailboat::sail_guard() - Sail not yet homed.\n");
            }
            last_not_home_msg = now;
        }
        if (sail_mode == MOTOR_SAIL || sail_mode == SAIL_ONLY || sail_mode == MOTOR_SOLAR) {
            // TODO: MM: do we need to home the mast when the failsafe triggers?
            // Only home the mast, if we are in a sail mode that automatically moves the sail

            if (mast_status.homed != AP_HAL::SERVO_HOMED && (tilt_imu == 0 || mast_status.pwm == 0)) {
                // Mast not homed and tilt sensor not configured, or not yet seen
                if (sail_status.moving) {
                    // Sail is moving
                    printf("ERROR: The mast isn't homed, but the sail is moving.\n");
                    gcs().send_text(MAV_SEVERITY_ERROR, "Sail moving before mast is homed.");
                } else if (wind_strength == WIND_HIGH) {
                    // The mast cannot be homed in high winds, so do nothing.
                } else if (rover.g2.sailboat.tilt_imu != 0) {
                    // If we're using a tilt sensor, we don't home the mast
                } else if (mast_status.homed != AP_HAL::SERVO_HOMING) {
                    gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Homing the mast.");
                    // Home the mast
                    AP_HAL::get_HAL().rcout->home(MAST_SERVO_CH-1);
                }
            } else {
                // Sail not homed
        //printf("Rover::sail_guard() - Sail not yet homed.\n");
                if (mast_status.moving) {
                    // Mast is homed but moving, do nothing
                } else if (mast_status.pwm < (1900 - tilt_err)) {
                    // Mast is not up
                    if (wind_strength == WIND_HIGH) {
                        // Wait for wind to drop
                    } else {
                        // raise mast before homing the sail
                        gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Raising the mast to home the sail.");
                        set_mast_position(1900);
                    }
                } else if (sail_status.homed != AP_HAL::SERVO_HOMING) {
                    gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Homing the sail.");
                    // Home the mast
                    AP_HAL::get_HAL().rcout->home(SAIL_SERVO_CH-1);
                }
            }
        }
        return;
    }

    if (wind_strength >= WIND_HIGH) {
        // Have high winds, stow the sail
//        DEBUGV("Rover::sail_guard() - Stow sail\n");
        if (!stowing_sail) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Sailboat: High winds, stowing sail.");
        }
        stowing_sail = true;

        if (!(sail_status.moving) && (abs(sail_status.pwm - 1500) <= sail_stow_error)) {
            // Sail centered and stopped, lower mast

            if (mast_set_pos > (1100 + 10) ||
                    (!mast_status.moving && mast_status.pwm > (1100 + tilt_err))) {
                // mast has not been told to lower, or mast has stopped moving and is not down
                set_mast_position(1100);
            }
        } else {
            // Center the sail, before lowering
            set_sail_position(1500);
        }
        return;
    } else if (sail_mode == MOTOR_SOLAR || // TODO: in motor solar only raise sail if sun is up
            (wind_strength > WIND_LOW && (sail_mode == MOTOR_SAIL || sail_mode == SAIL_ONLY))) {
        if (mast_status.pwm < (1900 - tilt_err) || mast_set_pos < (1900 - 10)) {
//            DEBUGV("Rover::sail_guard() - Raising the sail.\n");
            // We are in a mode that uses the sail, so raise it
            if (mast_set_pos < (1900 - 10) || !mast_status.moving || stowing_sail) {
                // If the mast isn't moving or the mast set position is not UP, then we raise the sail
                // Also raise the sail if it is being stowed
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Raising the mast for sailing. (%d)",
                        (int)mast_set_pos);
                stowing_sail = false;
                set_mast_position(1900);
            }
        } else if (stowing_sail) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Sailboat: Stopping stowing of mast, no longer required. (%d)",
                    (int)mast_set_pos);
            stowing_sail = false;
        }
    }
}

const AP_Param::GroupInfo Winch::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Winch
    // @Description: This enables Winch functionality
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Winch, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ENCODER_IN
    // @DisplayName: Winch encoder (retracted)
    // @Description: Winch encoder reading when the cable is fully retracted
    // @Units: ticks
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ENCODE_IN", 2, Winch, encoder_in, 0),

    // @Param: ENCODER_OUT
    // @DisplayName: Winch encoder (deployed)
    // @Description: Winch encoder reading when the cable is fully deployed
    // @Units: ticks
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ENCODE_OUT", 3, Winch, encoder_out, 0),

    AP_GROUPEND
};

const AP_Param::GroupInfo NMEA2k_Params::var_info[] = {
    // @Param: GPS_1
    // @DisplayName: Primary GPS ID
    // @Description: The NMEA2k ID of the primary GPS device.
    // @Units:
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GPS_1", 1, NMEA2k_Params, gps_1, 0),

    // @Param: GPS_2
    // @DisplayName: Secondary GPS ID
    // @Description: The NMEA2k ID of the secondary GPS device.
    // @Units:
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GPS_2", 2, NMEA2k_Params, gps_2, 0),

    // @Param: GPS_3
    // @DisplayName: Tertiary GPS ID
    // @Description: The NMEA2k ID of the tertiary GPS device.
    // @Units:
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GPS_3", 3, NMEA2k_Params, gps_3, 0),

    // @Param: FILT_COG
    // @DisplayName: COG filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_COG", 4, NMEA2k_Params, filt_cog, 5),

    // @Param: FILT_SOG
    // @DisplayName: SOG filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_SOG", 5, NMEA2k_Params, filt_sog, 5),

    // @Param: FILT_BSP
    // @DisplayName: Boatspeed filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_BSP", 6, NMEA2k_Params, filt_bsp, 5),

    // @Param: FILT_LEE
    // @DisplayName: Leeway filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_LEE", 7, NMEA2k_Params, filt_lee, 5),

    // @Param: FILT_HDG_TC
    // @DisplayName: Heading filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_HDG_TC", 8, NMEA2k_Params, filt_hdg_tc, 5),

    // @Param: FILT_HDG_NL
    // @DisplayName: Heading filter non-linear constant
    // @Description: Non-linear constant for exponential filter.
    // @Units: Degrees
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_HDG_NL", 9, NMEA2k_Params, filt_hdg_nl, 5),

    // @Param: FILT_TWS
    // @DisplayName: TWS filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_TWS", 10, NMEA2k_Params, filt_tws, 5),

    // @Param: FILT_TWD
    // @DisplayName: TWD filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_TWD", 11, NMEA2k_Params, filt_twd, 5),

    // @Param: FILT_TWA
    // @DisplayName: TWA filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_TWA", 12, NMEA2k_Params, filt_twa, 5),

    // @Param: FILT_AWA
    // @DisplayName: AWA filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_AWA", 13, NMEA2k_Params, filt_awa, 5),

    // @Param: FILT_AWS
    // @DisplayName: AWS filter time constant
    // @Description: Seconds time constant for exponential filter.
    // @Units: Seconds
    // @Range: ...
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("F_AWS", 14, NMEA2k_Params, filt_aws, 5),

    // @Param: COMPASS_1
    // @DisplayName: Primary compass ID
    // @Description: ID of the primary compass.
    // @Range: 0..255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COMPASS_1", 15, NMEA2k_Params, compass_1, 0),

    // @Param: COMPASS_2
    // @DisplayName: Secondary compass ID
    // @Description: ID of the secondary compass.
    // @Range: 0..255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COMPASS_2", 16, NMEA2k_Params, compass_2, 0),

    // @Param: USE_FILT
    // @DisplayName: Use Filtered NMEA2K Input
    // @Description: Whether or not to feed the filtered nmea2k reading into the autopilot.
    // @Range: 0..1
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("USE_FILT", 17, NMEA2k_Params, use_filtered, 0),

    // @Param: WATER_DEP
    // @DisplayName: Water depth ID
    // @Description: ID of the sensor for water depth
    // @Range: 0..255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WATER_DP", 18, NMEA2k_Params, water_depth, 0),

    // @Param: WATER_SPD
    // @DisplayName: Water speed ID
    // @Description: ID of the sensor for water speed
    // @Range: 0..255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("WATER_SP", 19, NMEA2k_Params, water_speed, 0),

    AP_GROUPEND
};
