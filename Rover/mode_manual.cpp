#include "mode.h"
#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    if (!rover.arming.is_armed()) {
        // Vehicle is disarmed, do nothing.
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    if (channel_throttle->get_radio_in() == 0 ||
            channel_steer->get_radio_in() == 0 ||
            channel_lateral->get_radio_in() == 0) {
        // Receiver has not seen the remote yet. Do nothing.
        return;
    }

    float desired_steering, desired_throttle, desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }

    // walking robots support roll, pitch and walking_height
    float desired_roll, desired_pitch, desired_walking_height;
    get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
    get_pilot_desired_walking_height(desired_walking_height);
    g2.motors.set_roll(desired_roll);
    g2.motors.set_pitch(desired_pitch);
    g2.motors.set_walking_height(desired_walking_height);

    // set sailboat sails
    float desired_mainsail;
    float desired_wingsail;
    float desired_mast_rotation;
    g2.sailboat.get_pilot_desired_mainsail(desired_mainsail, desired_wingsail, desired_mast_rotation);
    g2.motors.set_mainsail(desired_mainsail);
    g2.motors.set_wingsail(desired_wingsail);
    g2.motors.set_mast_rotation(desired_wingsail);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, false);
    g2.motors.set_lateral(desired_lateral);

    if (rover.is_boat() && rover.g2.sailboat.sail_enabled() && rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        // If on a Bluebottle in a sailing or solar mode, the sail automatically adjusts
        switch (rover.g2.sailboat.get_sail_mode()) {
        case Sailboat::MOTOR_SAIL: case Sailboat::SAIL_ONLY: case Sailboat::MOTOR_SOLAR:
        {{
            uint16_t pwm = g2.sailboat.get_optimal_sail_position();
            if (pwm != 0) {
                g2.sailboat.set_sail_position(pwm);
            }
        }}
            break;

        case MOTOR_ONLY: case WAVE_POWER:
            break;
        }
    }
}
