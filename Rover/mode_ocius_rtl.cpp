#include "mode.h"
#include "Rover.h"

bool ModeOciusRTL::_enter(ModeReason reason)
{
    // TODO: find index of sequence start WP
    AP_Mission *mission = AP::mission();
    if (mission == NULL) {
        return false;
    }

    _desired_throttle = constrain_float(g2.rtl_throttle, 0.0, 100.0);

    rtl_wp_idx = -1;
    Location wp = {};
    float dist = INFINITY;
    float next_leg_bearing_cd = AR_WPNAV_HEADING_UNKNOWN;

    Location cloc = rover.current_loc;

    uint16_t num_commands = mission->num_commands();
    rover.gcs().send_text(MAV_SEVERITY_DEBUG, "%u commands in mission.", (uint32_t)num_commands);

    AP_Mission::Mission_Command cmd;
    for (uint16_t i = 0; i < num_commands; ++i) {
        if (mission->read_cmd_from_storage(i, cmd)) {
            if (rtl_wp_idx == -1) {
                if (cmd.id != MAV_CMD_WAYPOINT_SEQUENCE_START || cmd.p1 != 0)
                    continue;
                rover.gcs().send_text(MAV_SEVERITY_DEBUG, "Magic index in mission is %u.", (uint32_t)i);
                rtl_wp_idx = i;
                wp = cmd.content.location;
                dist = cloc.get_distance(wp);
            } else if (AP_Mission::is_nav_cmd(cmd)) {
                switch (cmd.id) {
                case MAV_CMD_NAV_WAYPOINT:
                case MAV_CMD_NAV_LOITER_UNLIM:
                case MAV_CMD_NAV_LOITER_TURNS:
                case MAV_CMD_NAV_LOITER_TIME:
                case MAV_CMD_WAYPOINT_SEQUENCE_START:
                    {
                        Location loc2 = cmd.content.location;
                        float d2 = cloc.get_distance(loc2);

                        if (d2 <= dist) {
                            rtl_wp_idx = i;
                            wp = loc2;
                            dist = d2;
                        }
                    }
                    break;
                }
            }
        }
    }

    rover.gcs().send_text(MAV_SEVERITY_DEBUG, "Target RTL index is %u.", (uint32_t)rtl_wp_idx);

    if (rtl_wp_idx == -1) {
        // TODO: set home as the desired location
        wp = AP::ahrs().get_home();
        rover.gcs().send_text(MAV_SEVERITY_DEBUG, "RTL index using home location %d,%d.", wp.lat,wp.lng);
    } else {
        next_leg_bearing_cd = mission->get_next_ground_course_cd(AR_WPNAV_HEADING_UNKNOWN);
    }

    next_wp = wp;
    if (!g2.wp_nav.set_desired_location(next_wp, cloc, next_leg_bearing_cd)) {
        rover.gcs().send_text(MAV_SEVERITY_ERROR, "RTL unable to set desired location.");
        return false;
    }

//    bool have_next_wp = mission->get_next_nav_cmd(rtl_wp_idx+1, cmd);
//    bool stop_at_wp = rtl_wp_idx == -1 || !have_next_wp;
//    if (stop_at_wp) {
//        g2.wp_nav.set_s
//
//        // set desired location to reasonable stopping point
//        if (!g2.wp_nav.set_desired_location_to_stopping_location()) {
//            return false;
//        }
//    }

    // TODO: add rtl_throttle param
    // initialise waypoint speed
    if (is_positive(g2.rtl_speed)) {
        g2.wp_nav.set_desired_speed(g2.rtl_speed);
    } else {
        g2.wp_nav.set_desired_speed_to_default();
    }
    return true;
}

void ModeOciusRTL::update()
{
    // update distance to destination
    _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());

    _desired_throttle = constrain_float(g2.rtl_throttle, 0.0, 100.0);

    bool location_is_end = (rtl_wp_idx == -1);
    if (!g2.wp_nav.reached_destination()) {
        // update navigation controller
        navigate_to_waypoint();
    } else {
        if (rtl_wp_idx != -1) {
            AP_Mission *mission = AP::mission();
            if (mission == NULL) {
                return;
            }

            AP_Mission::Mission_Command cmd;
            uint16_t num_commands = mission->num_commands();
            int32_t old_rtl_idx = rtl_wp_idx;
            for (uint16_t i = rtl_wp_idx+1; i < num_commands; ++i) {
                if (mission->read_cmd_from_storage(i, cmd)) {
                    if (AP_Mission::is_nav_cmd(cmd)) {
                        if (cmd.id == MAV_CMD_NAV_WAYPOINT || cmd.id == MAV_CMD_NAV_LOITER_UNLIM ||
                                cmd.id == MAV_CMD_NAV_LOITER_TURNS || cmd.id == MAV_CMD_NAV_LOITER_TIME ||
                                cmd.id == MAV_CMD_WAYPOINT_SEQUENCE_START)
                        {
                            rtl_wp_idx = i;

                            int32_t next_leg_bearing_cd = mission->get_next_ground_course_cd(AR_WPNAV_HEADING_UNKNOWN);
                            if (g2.wp_nav.set_desired_location(cmd.content.location, next_wp, next_leg_bearing_cd)) {
                                next_wp = cmd.content.location;
                            }

                            break;
                        }
                    }
                }
            }
            location_is_end = (old_rtl_idx == rtl_wp_idx);
        }

        if (location_is_end) {
            // we have reached the destination so stay here
            if (_distance_to_destination > g2.sailboat.hold_radius) {
                if (g2.wp_nav.set_desired_location(next_wp)) {
                    g2.wp_nav.set_reversed(false);
                    navigate_to_waypoint();
                }
            } else if (rover.is_boat()) {
                stop_vehicle();
            } else {
                stop_vehicle();
            }
        } else {
            navigate_to_waypoint();
        }
    }
}

// get desired location
bool ModeOciusRTL::get_desired_location(Location& destination) const
{
    if (g2.wp_nav.is_destination_valid()) {
        destination = g2.wp_nav.get_destination();
        return true;
    }

    // should never reach here but just in case
    return false;
}

// return true if vehicle has reached or even passed destination
bool ModeOciusRTL::reached_destination() const
{
    return g2.wp_nav.reached_destination();
}
