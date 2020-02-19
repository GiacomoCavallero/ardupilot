#include "mode.h"
#include "Rover.h"
#include <GCS_MAVLink/GCS.h>

// enter this mode, returns false if we failed to enter
bool ModeHold::_enter(mode_reason_t reason) {
    // Need to get the end location from guided or auto modes
    if (reason == MODE_REASON_MISSION_END) {
        if (!rover.control_mode->get_desired_location(hold_wp)) {
            hold_wp = rover.current_loc;
        }
    } else {
        hold_wp = rover.current_loc;
    }
    figure8_idx = -1;
    figure8.clear();
    _reached_destination = true;
    return true;
}

float ModeHold::get_distance_to_destination() const {
    return rover.current_loc.get_distance(hold_wp);
}

float ModeHold::wp_bearing() const {
    return rover.current_loc.get_bearing_to(hold_wp) * 0.01f;
}

float ModeHold::nav_bearing() const {
    // return bearing to intermediate waypoints
    if (g2.sailboat.hold_mode == Hold_Figure8  && figure8_idx != -1) {
        return rover.current_loc.get_bearing_to(figure8[figure8_idx]) * 0.01f;
    }
    return wp_bearing();
}

bool ModeHold::get_desired_location(Location& destination) const {
    destination = hold_wp;
    return true;
}

bool ModeHold::set_desired_location(const struct Location& destination, float next_leg_bearing_cd) {
    hold_wp = destination;
    return true;
}

void ModeHold::update()
{
    float throttle = 0.0f;

    // if at current WP, advance the index
//    _distance_to_destination = rover.current_loc.get_distance(hold_wp);

    if (rover.is_boat() && g2.sailboat.hold_radius > 0) {
        float dist_to_wp = get_distance_to_destination();

        switch (g2.sailboat.hold_mode) {
        case Hold_Drift:
            figure8.clear();
            if (dist_to_wp <= g2.wp_nav.get_radius()) {
                _reached_destination = true;
            } else if (dist_to_wp > g2.sailboat.hold_radius) {
                _reached_destination = false;
            }
            if (_reached_destination) {
                // we've reached the destination. so drift
                g2.motors.set_mainsail(100.0f);

                // hold position - stop motors and center steering
                g2.motors.set_throttle(throttle);
                g2.motors.set_steering(0.0f);
            } else {
                // Need to return to the hold point.
                if (g2.wp_nav.set_desired_location(hold_wp)) {
                    g2.wp_nav.set_reversed(false);
                    navigate_to_waypoint();
                    return;
                }
            }
            break;
        case Hold_Active:
            figure8.clear();
            if (dist_to_wp <= g2.wp_nav.get_radius()) {
                _reached_destination = true;
            } else if (dist_to_wp > rover.g2.sailboat.hold_radius) {
                _reached_destination = false;
            }
            if (g2.wp_nav.set_desired_location(hold_wp)) {
                g2.wp_nav.set_reversed(false);
                if (_reached_destination) {
                    // we've reached the destination. so maintain 0 velocity
                    _desired_yaw_cd = wp_bearing();

                    // run steering and throttle controllers
                    calc_steering_to_heading(_desired_yaw_cd);
                    calc_throttle(0, false);
                } else {
                    // Need to return to the hold point.
                    g2.wp_nav.set_desired_speed(g2.wp_nav.get_default_speed());
                }
                navigate_to_waypoint();
                return;
            }
            break;
        case Hold_Figure8:

            // calculate the 4 points of the figure 8.
            float true_wind = g2.windvane.get_true_wind_direction_rad();
            true_wind = wrap_360(degrees(true_wind));
            if ((rover.is_boat() && g2.sailboat.sail_enabled() && g2.sailboat.sail_is_safe()) || figure8.size() == 0) {
                bool regenFigure8 = true;
                if (figure8_idx != -1) {
                    Location destination = figure8[figure8_idx];
                    float dist_to_leaf = rover.current_loc.get_distance(destination);
                    float bearing_to_leaf = rover.current_loc.get_bearing_to(destination) * 0.01f;
                    float angle_from_wind = wrap_180(bearing_to_leaf - true_wind);

                    if (dist_to_leaf < g2.sailboat.hold_radius && fabs(angle_from_wind) >= g2.sailboat.sail_no_go) {
                        // As the vehicle approaches the WP, stop moving it, because small wind changes can swing the desired bearing massively
                        regenFigure8 = false;
                    }
                }
                if (regenFigure8) {
                    figure8 = generateFigure8(hold_wp, 4, true_wind, g2.sailboat.hold_radius, g2.wp_nav.get_radius());
                }
            }

            if (figure8_idx != -1) {
                // check if we've reached the destination
                if (g2.wp_nav.reached_destination()) {
                    figure8_idx = (figure8_idx + 1) % figure8.size();
//                    printf("ModeHold::update() - Figure 8 - reached end of leaf, redirecting to idx %Zd.\n",
//                            figure8_idx);
                }

                Location destination = figure8[figure8_idx];

                // if current WP is up wind clear index
                destination = figure8[figure8_idx];
                float bearing_to_dest = rover.current_loc.get_bearing_to(destination) * 0.01f;
                float angle_from_wind = wrap_180(bearing_to_dest - true_wind);

                if (rover.is_boat() && g2.sailboat.sail_enabled() && fabs(angle_from_wind) < g2.sailboat.sail_no_go) {
                    // current WP too close to wind, need to reselect
//                    printf("ModeHold::update() - Figure 8 - WP too close to wind, clearing.\n");
                    figure8_idx = -1;
                }
            }

            // if no selected WP, find a WP on the figure 8 that is not upwind +/- 5 degrees
            if (figure8_idx == -1) {
                figure8_idx = get_preferred_leaf_idx(true_wind);
            }

            if (figure8_idx != -1) {
                // Have a valid leaf point, navigate to it
                _reached_destination = true;
                Location destination = figure8[figure8_idx];
                Location origin = figure8[(figure8_idx + figure8.size() - 1) % figure8.size()];
                if (!g2.wp_nav.set_desired_location(destination, origin)) {
                    hal.console->printf("ModeHold: Unable to set figure 8 waypoint.\n");
                }
//                printf("ModeHold::update() - Figure 8 - navigating to WP index %Zd.\n", figure8_idx);
            } else {
                // We have drifted too far downwind, navigate to the WP
                // TODO: should this only be done once??
//                printf("ModeHold::update() - Figure 8 - returning to hold location.\n");
                if (reached_destination()) {
                    gcs().send_text(MAV_SEVERITY_INFO, "ModeHold - Drifted off waypoint returning to hold location.");
                }
                if (!g2.wp_nav.set_desired_location(hold_wp, rover.current_loc)) {
                    hal.console->printf("ModeHold: Unable to navigate to center of figure 8.\n");
                }
                _reached_destination = false;
//                printf("ModeHold::update() - Figure 8 - returning to hold location.\n");
            }
            navigate_to_waypoint();
            return;

            break;
        }
    }

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle);
    }

    // relax mainsail
    g2.motors.set_mainsail(100.0f);
    g2.motors.set_wingsail(0.0f);

    // hold position - stop motors and center steering
    g2.motors.set_throttle(throttle);
    g2.motors.set_steering(0.0f);
}

static float prev_true_wind = INFINITY;

#define DIST_2_JIBE_WP      5   // TODO: make a parameter
std::vector< Location > ModeHold::generateFigure8(const Location& centre, int num_points, float true_wind, float hold_radius, float wp_radius) {
    std::vector< Location > rval;
    if (true_wind == INFINITY) {
        true_wind = g2.windvane.get_true_wind_direction_rad();
        true_wind = wrap_180(degrees(true_wind));
    }
    if (fabs(prev_true_wind - true_wind) > 1) {
//        printf("ModeHold::generateFigure8 - generating figure8 for true_wind %.2f\n", true_wind);
        prev_true_wind = true_wind;
    }
    if (num_points <= 5) {
        float angle_from_90 = ToDeg((float)atan2(wp_radius + DIST_2_JIBE_WP, hold_radius));
//        rval.push_back(centre);

        std::vector <float> bearings;
        bearings.push_back(CLIP_360(true_wind + (90 - angle_from_90)));
        bearings.push_back(CLIP_360(true_wind + (90 + angle_from_90)));
        bearings.push_back(CLIP_360(true_wind - (90 - angle_from_90)));
        bearings.push_back(CLIP_360(true_wind - (90 + angle_from_90)));

        for (size_t i = 0; i < bearings.size(); ++i) {
            Location l = centre;
            l.offset_bearing(bearings[i], hold_radius);
            rval.push_back(l);
        }
    } else {
        // TODO: change algorithm, find the ends then draw circle segments
        for (int i = 0; i < num_points; ++i) {
            float X = sin(M_PI * 2 * i / (num_points - 1));
            float Y = sin(M_PI * 4 * i / (num_points - 1)) / 4;
            float vector_bearing = CLIP_360(ToDeg(atan2(-X,Y)));
            vector_bearing = CLIP_360(vector_bearing + true_wind);

            Location l = centre;
            l.offset_bearing(vector_bearing, sqrt(X*X+Y*Y) * hold_radius);

            rval.push_back(l);
        }
    }


//    printf("ModeHold::generateFigure8() - generated for true_wind: %.1f\n", true_wind);
//    for (size_t i = 0; i < rval.size(); ++i) {
//        printf("\t%2Zu: %.7f, %.7f\n", i, rval[i].lat * 1e-7f, rval[i].lng * 1e-7f);
//    }
    return rval;
}

#define FIGURE8_SELECT_ERROR_MARGIN     5
ssize_t ModeHold::get_preferred_leaf_idx(float true_wind) {
    if (figure8.size() < 4) return -1;  // This should be unnecessary.

    // Search order 0, 2, 1, 3.  Need downwind and closest to current heading.
    ssize_t rval = -1;
    float turn_angle = INFINITY;
    float current_heading = ahrs.yaw_sensor * 0.01f;

    Location leaf_wp = figure8[0];
    float bearing_to_dest = rover.current_loc.get_bearing_to(leaf_wp) * 0.01f;
    float angle_from_wind = wrap_180(bearing_to_dest - true_wind);

    if (fabs(angle_from_wind) >= (g2.sailboat.sail_no_go + FIGURE8_SELECT_ERROR_MARGIN)) {
        // Leaf WP is downwind with an error margin.
        rval = 0;
        turn_angle = current_heading - bearing_to_dest;
        turn_angle = fabs(wrap_180(turn_angle));
    }

    leaf_wp = figure8[2];
    bearing_to_dest = rover.current_loc.get_bearing_to(leaf_wp) * 0.01f;
    angle_from_wind = wrap_180(bearing_to_dest - true_wind);

    if (fabs(angle_from_wind) >= (g2.sailboat.sail_no_go + FIGURE8_SELECT_ERROR_MARGIN)) {
        // Leaf WP is downwind with an error margin.
        float my_turn_angle = current_heading - bearing_to_dest;
        my_turn_angle = fabs(wrap_180(my_turn_angle));

        if (my_turn_angle < turn_angle) {
            rval = 2;
            turn_angle = my_turn_angle;
        }
    }

    if (rval != -1) {
        // All upwind points take priority
        return rval;
    }

    leaf_wp = figure8[1];
    bearing_to_dest = rover.current_loc.get_bearing_to(leaf_wp) * 0.01f;
    angle_from_wind = wrap_180(bearing_to_dest - true_wind);

    if (fabs(angle_from_wind) >= (g2.sailboat.sail_no_go + FIGURE8_SELECT_ERROR_MARGIN)) {
        // Leaf WP is downwind with an error margin.
        rval = 1;
        turn_angle = current_heading - bearing_to_dest;
        turn_angle = fabs(wrap_180(turn_angle));
    }

    leaf_wp = figure8[3];
    bearing_to_dest = rover.current_loc.get_bearing_to(leaf_wp) * 0.01f;
    angle_from_wind = wrap_180(bearing_to_dest - true_wind);

    if (fabs(angle_from_wind) >= (g2.sailboat.sail_no_go + FIGURE8_SELECT_ERROR_MARGIN)) {
        // Leaf WP is downwind with an error margin.
        float my_turn_angle = current_heading - bearing_to_dest;
        my_turn_angle = fabs(wrap_180(my_turn_angle));

        if (my_turn_angle < turn_angle) {
            rval = 3;
            turn_angle = my_turn_angle;
        }
    }

    return rval;
}
