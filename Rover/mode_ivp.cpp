#include "mode.h"
#include "Rover.h"

bool ModeIVP::_enter(ModeReason reason)
{
    _reached_destination = false;
    have_attitude_target = false;
    _target_is_throttle = false;

    _max_distance = -1;

    return true;
}

void ModeIVP::update()
{
    // stop vehicle if target not updated within 3 seconds
    if (have_attitude_target && _max_distance < 0 && (millis() - _des_att_time_ms) > 5000)
    {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "IVP Mode: target not received in last 5 secs, stopping");
        have_attitude_target = false;
    }
    else if (have_attitude_target && _max_distance >= 0 && get_distance_to_destination() <= 0)
    {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "IVP Mode: target distance reached, stopping");
        have_attitude_target = false;
        // If we're a boat we drop to hold at the end of the travel
        if (rover.is_boat())
        {
            if (!rover.set_mode(rover.mode_hold, ModeReason::MISSION_END))
            {
                stop_vehicle();
            }
        }
    }
    if (have_attitude_target) {
        // run steering and throttle controllers
        // calc_steering_to_heading(_desired_yaw_cd);
        // call heading controller
        const float steering_out = attitude_control.get_steering_out_heading(radians(_desired_yaw_cd*0.01f),
                                                                             radians(0.0f),
                                                                             g2.motors.limit.steer_left,
                                                                             g2.motors.limit.steer_right,
                                                                             rover.G_Dt);
        set_steering(steering_out * 4500.0f, false);

        if (_target_is_throttle) {
            // IVP allows the sail to automatically adjust
            if (rover.g2.sailboat.sail_enabled()) {
                // sailboats use special throttle and mainsail controller.
                // but we ignore the throttle out
                float mainsail_out = 0.0f;
                float throttle_out = 0.0f;
                float wingsail_out = 0;
                float mast_rotation_out = 0;
                rover.g2.sailboat.get_throttle_and_mainsail_out(1.0, throttle_out, mainsail_out, wingsail_out, mast_rotation_out);
                rover.g2.motors.set_mainsail(mainsail_out);
            }
            g2.motors.set_throttle(_desired_speed);
        } else {
            float desired_speed = rover.g2.wp_nav.get_desired_speed();
            calc_throttle(calc_speed_nudge(desired_speed, is_negative(desired_speed)), true);
        }
    } else {
        // we are waiting for IVP to tell us direction and speed
        stop_vehicle();
    }
}

// set desired attitude
void ModeIVP::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // handle guided specific initialisation and logging
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;
    _max_distance = -1;

    // record targets
    _desired_yaw_cd = yaw_angle_cd;
    _desired_speed = target_speed;
    if (target_speed > 0)
        rover.g2.wp_nav.set_desired_speed(target_speed);
    _target_is_throttle = false;
    have_attitude_target = true;

    // log new target
    // ModeGuided::Guided_HeadingAndSpeed
    rover.Log_Write_GuidedTarget(1, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(target_speed, 0.0f, 0.0f));
}

// set desired attitude
void ModeIVP::set_desired_heading_and_throttle(float yaw_angle_cd, float target_throttle)
{
    // handle initialisation

    // handle guided specific initialisation and logging
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;
    _max_distance = -1;

    // record targets
    _desired_yaw_cd = yaw_angle_cd;
    _target_is_throttle = true;
    _desired_speed = target_throttle;

    have_attitude_target = true;

    // log new target
    // ModeGuided::Guided_HeadingAndSpeed
    rover.Log_Write_GuidedTarget(1, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(target_throttle*g2.wp_nav.get_default_speed(), 0.0f, 0.0f));
}

void ModeIVP::set_max_distance(float distance)
{
    _start_location = rover.current_loc;
    _max_distance = distance;
}

float ModeIVP::get_distance_to_destination() const
{
    if (_max_distance < 0)
        return 0;
    return _max_distance - _start_location.get_distance(rover.current_loc);
}

bool ModeIVP::get_desired_location(Location& destination) const
{
    if (_max_distance <= 0)
        return false;
    float dist = get_distance_to_destination();
    if (dist <= 0)
        return false;

    // TODO: this possibly should use the course over ground
    destination = rover.current_loc;
    destination.offset_bearing(_desired_yaw_cd/100, dist);
    return true;
}
