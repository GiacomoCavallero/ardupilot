#include "mode.h"
#include "Rover.h"

bool ModeIVP::_enter(mode_reason_t reason)
{
    _reached_destination = false;
    have_attitude_target = false;
    _target_is_throttle = false;

    return true;
}

void ModeIVP::update()
{
    // stop vehicle if target not updated within 3 seconds
    if (have_attitude_target && (millis() - _des_att_time_ms) > 3000) {
        gcs().send_text(MAV_SEVERITY_WARNING, "IVP Mode: target not received in last 3secs, stopping");
        have_attitude_target = false;
    }
    if (have_attitude_target) {
        // run steering and throttle controllers
        calc_steering_to_heading(_desired_yaw_cd);

        if (_target_is_throttle) {
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
    // call parent
    Mode::set_desired_heading_and_speed(yaw_angle_cd, target_speed);

    // handle guided specific initialisation and logging
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;

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

    // record targets
    _desired_yaw_cd = yaw_angle_cd;
    _target_is_throttle = true;
    _desired_speed = target_throttle;

    have_attitude_target = true;

    // log new target
    // ModeGuided::Guided_HeadingAndSpeed
    rover.Log_Write_GuidedTarget(1, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(target_throttle*g2.wp_nav.get_default_speed(), 0.0f, 0.0f));
}
