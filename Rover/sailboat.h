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
    Rover Sailboat functionality
*/

#include "ocius.h"

#define KNOTS_PER_METRE     1.94384

#define RUDDER_SERVO_CH     1
#define MAST_SERVO_CH       9
#define SAIL_SERVO_CH       10
#define WINCH_SERVO_CH      14

class Sailboat
{
public:

    // constructor
    Sailboat();

    // enabled
    bool sail_enabled() const;

    // true if sailboat navigation (aka tacking) is enabled
    bool tack_enabled() const;

    // setup
    void init();

    // initialise rc input (channel_mainsail)
    void init_rc_in();

    // decode pilot mainsail input and return in steer_out and throttle_out arguments
    // mainsail_out is in the range 0 to 100, defaults to 100 (fully relaxed) if no input configured
    // wingsail_out is in the range -100 to 100, defaults to 0
    // mast_rotation_out is in the range -100 to 100, defaults to 0
    void get_pilot_desired_mainsail(float &mainsail_out, float &wingsail_out, float &mast_rotation_out);

    // calculate throttle and mainsail angle required to attain desired speed (in m/s)
    void get_throttle_and_mainsail_out(float desired_speed, float &throttle_out, float &mainsail_out, float &wingsail_out, float &mast_rotation_out);

    // Velocity Made Good, this is the speed we are traveling towards the desired destination
    float get_VMG() const;

    // handle user initiated tack while in acro mode
    void handle_tack_request_acro();

    // return target heading in radians when tacking (only used in acro)
    float get_tack_heading_rad();

    // handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
    void handle_tack_request_auto();

    // clear tacking state variables
    void clear_tack();

    // returns true if boat is currently tacking
    bool tacking() const;

    // returns true if sailboat should take a indirect navigation route to go upwind
    bool use_indirect_route(float desired_heading_cd) const;

    // calculate the heading to sail on if we cant go upwind
    float calc_heading(float desired_heading_cd);

    // states of USE_MOTOR parameter and motor_state variable
    enum class UseMotor {
        USE_MOTOR_NEVER  = 0,
        USE_MOTOR_ASSIST = 1,
        USE_MOTOR_ALWAYS = 2
    };

    enum SailMode {
        MOTOR_ONLY      = 0,
        MOTOR_SAIL      = 1,
        SAIL_ONLY       = 2,
        WAVE_POWER      = 3,
        MOTOR_SOLAR     = 5,
    };

    enum SailFlags {
        FLAG_NONE           = 0,
        FLAG_JIBE_ONLY      = 1,
        FLAG_IGNORE_XTRACK  = 2,  // Replaced with NAVL1_IGN_XTRACK
        FLAG_HEADING_TO_WP  = 4,
    };

    enum WindStrength {
        WIND_UNKNOWN        = 0,  // No reading from weather vane
        WIND_LOW            = 1,  // Wind too low for sailing
        WIND_FAIR           = 2,  // Light sailing wind
        WIND_STRONG         = 4,  // Strong sailing wind
        WIND_HIGH           = 8,  // High winds, stow sail if possible

        WIND_GOOD           = 6,  // Wind is fair/strong, so can sail
    };

//    enum SailHoldMode {
//        HOLD_DRIFT      = 0,
//        HOLD_ACTIVE     = 1,
//        HOLD_FIGURE8    = 2,
//    };

    // set state of motor
    // if report_failure is true a message will be sent to all GCSs
    void set_motor_state(UseMotor state, bool report_failure = true);

    // set the sailing mode
    void set_sail_mode(SailMode sailmode);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // return sailboat loiter radius
    float get_loiter_radius() const {return loit_radius;}

    MAV_RESULT set_servo(uint8_t channel, uint16_t pwm, bool gcs_command = false);
    MAV_RESULT set_mast_position(uint16_t pwm, bool gcs_command = false);
    MAV_RESULT set_sail_position(uint16_t pwm, bool gcs_command = false);
    MAV_RESULT set_winch_position(uint16_t pwm, bool gcs_command = false);
    bool sail_is_safe(bool gcs_command = false) const;
    uint16_t get_optimal_sail_position() const;
    void check_wind();
    void sail_guard();
    SailMode get_sail_mode() const { return (SailMode)sail_mode.get(); }
private:

    // true if motor is on to assist with slow tack
    bool motor_assist_tack() const;

    // true if motor should be on to assist with low wind
    bool motor_assist_low_wind() const;

    // parameters
    AP_Int8 enable;
    AP_Float sail_angle_min;
    AP_Float sail_angle_max;
    AP_Float sail_angle_ideal;
    AP_Float sail_angle_error;
    AP_Float sail_heel_angle_max;
    AP_Float sail_no_go;
    AP_Float sail_windspeed_min;
    AP_Float xtrack_max;
    AP_Float loit_radius;
    AP_Float sail_windspeed_max;
    AP_Int16 sail_stow_error;
    AP_Int8 sail_mode;
    AP_Int16 sail_flags;
    AP_Int32 sail_epos_zero;
    AP_Float sail_tack_corridor;
    AP_Int32 mast_time_up;
    AP_Int32 mast_time_down;
    AP_Int32 mast_time_delay;
    AP_Int16 tilt_imu;
    AP_Int16 tilt_err;
    AP_Float tilt_filt;

    AP_Int8 hold_mode;
    AP_Float hold_radius;

    RC_Channel *channel_mainsail;   // rc input channel for controlling mainsail
    bool currently_tacking;         // true when sailboat is in the process of tacking to a new heading
    float tack_heading_rad;         // target heading in radians while tacking in either acro or autonomous modes
    uint32_t tack_request_ms;       // system time user requested tack
    uint32_t auto_tack_start_ms;    // system time when tack was started in autonomous mode
    uint32_t tack_clear_ms;         // system time when tack was cleared
    bool tack_assist;               // true if we should use some throttle to assist tack
    UseMotor motor_state;           // current state of motor output

    bool stowing_sail;
//    NMEA2K nmea2k_sensors;
//    EncodedServo rudder, sail, mast, winch;
    WindStrength wind_strength;

    friend class GCS_MAVLINK_Rover;
    friend class Rover;
    friend class RCOutput_Ocius;
    friend class Mode;
    friend class ModeHold;
    friend class ModeOciusRTL;
};

class Winch {
public:
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // parameters
    AP_Int8 enable;
    AP_Int32 encoder_in;
    AP_Int32 encoder_out;
};

class NMEA2k_Params {
public:
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // device ids
    AP_Int32 gps_1;
    AP_Int32 gps_2;
    AP_Int32 gps_3;
    AP_Int32 compass_1;
    AP_Int32 compass_2;
//    AP_Int32 wind_1;
    AP_Int32 water_depth;
    AP_Int32 water_speed;

    // Filtering parameters
    AP_Float filt_cog;
    AP_Float filt_sog;
    AP_Float filt_bsp;
    AP_Float filt_lee;
    AP_Float filt_hdg_tc;
    AP_Float filt_hdg_nl;
    AP_Float filt_tws;
    AP_Float filt_twd;
    AP_Float filt_aws;
    AP_Float filt_awa;
    AP_Float filt_twa;

    AP_Int8  use_filtered;
};
