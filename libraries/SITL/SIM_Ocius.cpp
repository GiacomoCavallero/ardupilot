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
  rover simulator class
*/

#include "SIM_Ocius.h"

#include <string.h>
#include <stdio.h>

//TODO: Need to provide sensor data for wind/water speed and direction
//#include <AP_GPS/AP_GPS_OCIUSN2K.h>

namespace SITL {

#define CLIP_180(A)     ((A) < -180? (A) + 360: (A) > 180? (A) - 360: (A))
#define CLIP_360(A)     ((A) < 0? (A) + 360: (A) > 360? (A) - 360: (A))

#define KNOTS_PER_METRE     1.94384

#define PWM_2_FLOAT(PWM)      ((float)((PWM - 1100) / 800.0F))
#define FLOAT_2_PWM(REL)      ((int)(REL * 800) + 1100)

//#define DEBUGV(fmt, ...) printf(fmt, ##__VA_ARGS__);gcs().send_text(MAV_SEVERITY_INFO, fmt, ##__VA_ARGS__);
//#define DEBUGV(fmt, ...) printf(fmt, ##__VA_ARGS__);
#define DEBUGV(fmt, ...)

SimOcius::SimOcius(const char *frame_str) :
    SimRover(frame_str),
    rudder(radians(-45), radians(45), 0.15),
    mast(radians(-90), 0, 0.30, radians(-90)),
    wing_sail(radians(-90), radians(90), 0.10),
    winch(0, radians(360 * 30), 2),
    is_wamv(false)
{
    // TODO: SimOcius::SimOcius() - Add default configurations
    is_wamv = (strcasecmp(frame_str, "wamv") == 0);
}

#define     OPTIMAL_LIFT_ANGLE      35
#define     STALL_ANGLE             50

/*
  update the rover simulation by one time step
 */
void SimOcius::update(const struct sitl_input &input)
{
    DEBUGV("SimOcius:: Updating simulator state.\n");
    float steering, throttle;


    // The Wam-V uses skid steering
    if (is_wamv) {
        float motor1 = -2 * ((input.servos[0] - 1000) / 1000.0f - 0.5f);
        float motor2 = -2 * ((input.servos[2] - 1000) / 1000.0f - 0.5f);
        steering = motor2 - motor1;
        throttle = 0.5*(motor1 + motor2);
    }
    else {
        steering = 2 * ((input.servos[0] - 1000) / 1000.0f - 0.5f);
        //        throttle = -2*((input.servos[2]-1000)/1000.0f - 0.5f);
        throttle = -(input.servos[2] - 1500) / 400.0f;
        DEBUGV("SimOcius:: servo 1: %u, servo 3: %u\n", input.servos[0], input.servos[2]);
    }

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;
    DEBUGV("SimOcius:: Have steering: %.1f, throttle: %.1f, delta time: %.6f\n", steering, throttle, delta_time);

    //if (input.debug[0] != 0.0 || input.debug[1] != 0.0)
    //{
    //    accel_body.zero();
    //    dcm.zero();
    //    gyro.zero();
    //    gyro_prev.zero();
    //    ang_accel.zero();

    //    velocity_ef.x = input.debug[0];
    //    velocity_ef.y = input.debug[1];
    //    velocity_ef.z = 0;
    //}
    //else
    //{
//    steering = -steering;

//    printf("SimOcius:: Sail Motor - pos: %.3f, target: %.3f, input: %.3f(%u)\n",
//            wing_sail.position, wing_sail.target, PWM_2_FLOAT(input.servos[9]), input.servos[9]);
//    printf("SimOcius:: Mast Motor - pos: %.3f, target: %.3f, input: %.3f(%u)\n",
//            mast.position, mast.target, PWM_2_FLOAT(input.servos[8]), input.servos[8]);

    // Update rudder/mast/sail positions
    rudder.setRelativeTarget((steering + 1) / 2);
    rudder.update(delta_time);
    if (input.servos[8])
        mast.setRelativeTarget(PWM_2_FLOAT(input.servos[8]));
    mast.update(delta_time);
    if (input.servos[9])
        wing_sail.setRelativeTarget(PWM_2_FLOAT(input.servos[9]));
    wing_sail.update(delta_time);
    if (input.servos[13])
        winch.setRelativeTarget(PWM_2_FLOAT(input.servos[13]));
    winch.update(delta_time);



    //    DEBUGV("SimOcius:: Sail sim: %d, %.2f, %d\n", input.servos[9], wing_sail.target, FLOAT_2_PWM(wing_sail.getRelativePosition()));
    //
    DEBUGV("SimOcius:: Mast: %d, Sail %d\n", input.servos[8], input.servos[9]);

    // use rudder position for the steering
    steering = -((rudder.getRelativePosition() * 2) - 1);

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;
    //    printf("Current velocity: %.3f\n", velocity_body.x);
    //    Matrix3f I = dcm * dcm.transposed();
    //    printf("[[ %.3f, %.3f, %.3f ]\n[ %.3f, %.3f, %.3f ]\n[ %.3f, %.3f, %.3f ]]\n",
    //            I.a.x, I.a.y, I.a.z, I.b.x, I.b.y, I.b.z, I.c.x, I.c.y, I.c.z);

        // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, speed);

    DEBUGV("SimOcius:: Have current speed: %.1f, direction: %.1f\n",
        input.water.speed, input.water.direction);
    if (isfinite(sitl->tide.speed) && isfinite(sitl->tide.direction)) {
        float curr_speed = sitl->tide.speed;
        float curr_dir = CLIP_360(sitl->tide.direction + 180.0);
        Vector3f vel_water = Vector3f(-curr_speed*cos(radians(curr_dir)), -curr_speed*sin(radians(curr_dir)), 0),
            rel_water = vel_water - velocity_ef;

        yaw_rate = calc_yaw_rate(steering, rel_water.length());
    }
    DEBUGV("SimOcius:: Have steering: %.1f, yaw_rate: %.1f\n", steering, yaw_rate);

    sitl_fdm fdm;
    fill_fdm(fdm);
    //    printf("Current headings: dcm: %.2f fdm: %.2f\n", fdm.yawDeg, (float)fdm.heading);

    //    printf("Yaw rate: %.2f(%.2f, %.2f, %.2f, %.2f)\n", yaw_rate, steering, speed, velocity_body.y, velocity_ef.length());

    gyro = Vector3f(0, 0, radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motor
    accel_body = Vector3f(0, 0, 0);

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

    //if we are a boat then we will drift if wind has a velocity or current has a velocity

    // Calculate hull water speed, calculate lateral water speed
    float rel_current_deg = CLIP_360(sitl->tide.direction - fdm.yawDeg + 180.0);
    double hull_water = velocity_body.x + sitl->tide.speed * cos(radians(rel_current_deg));
    double lateral_water = velocity_body.y + sitl->tide.speed * sin(radians(rel_current_deg));
    //        printf("Hdg: %.1f, Current Dir: %.1f Current Spd: %.1f \n", fdm.yawDeg, input.current.direction, input.current.speed);

/**    TODO: // Fake the sesor readings for wind and water speed/direction
    public_nmea2k_readings.wind_type = 0;
    public_nmea2k_readings.wind_speed = input.wind.speed * KNOTS_PER_METRE * 100;
    public_nmea2k_readings.wind_direction = input.wind.direction * 100;
    public_nmea2k_readings.water_speed = hull_water;
    DEBUGV("SimOcius:: Publishing sensors - wind %.1f, %.1f\n", public_nmea2k_readings.wind_speed, input.wind.direction);
**/
    // Simple change to speed up Wam-Vs
    if (is_wamv) hull_water /= 2;
    double drag_hull = (pow(1.005, fabs(hull_water * KNOTS_PER_METRE * hull_water * KNOTS_PER_METRE)) - 1) * 5;
    //        printf("h_w: %.2lf, knt_w: %.2lf, pow: %.3lf, d_h_fml: %.3lf, d_h: %.3lf\n",
    //                hull_water, hull_water * KNOTS_PER_METRE, pow(1.005, fabs(hull_water*KNOTS_PER_METRE * hull_water * KNOTS_PER_METRE)),
    //                (pow(1.005, fabs(hull_water * KNOTS_PER_METRE * hull_water * KNOTS_PER_METRE)) - 1) * 5, drag_hull);

    if (hull_water < 0) drag_hull = -drag_hull;
    //        printf("Hull speed: %.2lf, drag %.3lf\n", hull_water, drag_hull);
    double drag_lateral = (pow(1.1, fabs(lateral_water * KNOTS_PER_METRE)) - 1) * 20;
    if (lateral_water < 0) drag_lateral = -drag_lateral;
    //        printf("Lateral speed: %.2lf, drag %.3lf\n", lateral_water, drag_lateral);

    accel_body.x += throttle - drag_hull;
    accel_body.y -= drag_lateral;

    //        printf("Local Acceleration (%.3f, %.3f, %.3f)\n", accel_body.x, accel_body.y, accel_body.z);

    if (isfinite(input.wind.speed) && isfinite(input.wind.direction) && isfinite(velocity_ef.x) && isfinite(velocity_ef.y)) {
        float wind_speed = input.wind.speed;
        float wind_dir = input.wind.direction;
        Vector3f vel_wind = Vector3f(-wind_speed*cos(radians(wind_dir)), -wind_speed*sin(radians(wind_dir)), 0),
            rel_wind = vel_wind - velocity_ef;
        //
        //            printf("True wind(%.2f, %.2f, %.2f)\n", vel_wind.x, vel_wind.y, vel_wind.z);
        //            printf("Relative wind(%.2f, %.2f, %.2f)\n", rel_wind.x, rel_wind.y, rel_wind.z);
        wind_speed = rel_wind.length();
        wind_dir = degrees(atan2(-rel_wind.y, -rel_wind.x)) - fdm.yawDeg;
        wind_dir = CLIP_180(wind_dir);

        //            printf("Apparent Wind (%.1f deg, %.3f m/s), hdg: %.1lf\n", wind_dir, wind_speed, fdm.yawDeg);

        if (mast.position >= -M_PI_4) {
            // Calculate the lift & drag on the sail/wing
            float sail_area = fabs(cos(wing_sail.position - radians(wind_dir)));

            if (sail_area < 0.1) {
                // set a minimum value for sail exposure
                sail_area = 0.1;
            }

            //                printf("Exposed sail area: %.3f\n", sail_area);

            const float K_wind = 0.04;
            float wind_drag = sail_area * wind_speed * K_wind;

            accel_body -= Vector3f(wind_drag*cos(radians(wind_dir)), wind_drag*sin(radians(wind_dir)), 0);


            // Calculate the lift from the sail
            float attack_angle = 0;
            float sail_angle = degrees(-wing_sail.position);
            const float sail_default_attack = 5;
            if (wind_dir > 0) {
                attack_angle = wind_dir - 90 + sail_angle + sail_default_attack;
            }
            else {
                attack_angle = -wind_dir - 90 - sail_angle + sail_default_attack;
            }
            DEBUGV("SimOcius: apparent_wind: %.1f, sail_angle: %.1f, attack_angle: %.1f\n", wind_dir, sail_angle, attack_angle);

            //                printf("Attack angle: %.1f\n", attack_angle);

            float lift = 0;
            if (fabs(attack_angle) < OPTIMAL_LIFT_ANGLE) {
                lift = attack_angle / OPTIMAL_LIFT_ANGLE;
            }
            else if (fabs(attack_angle) < STALL_ANGLE) {
                lift = (STALL_ANGLE - fabs(attack_angle)) / (STALL_ANGLE - OPTIMAL_LIFT_ANGLE);
                if (attack_angle < 0)
                    lift = -lift;
                //                } else {
                //                    printf("Sail Stalled\n");
            }
            Vector3f wind_v = Vector3f(cos(radians(wind_dir)), sin(radians(wind_dir)), 0), lift_v;
            if (wind_v.y < 0) {
                lift_v.x = -wind_v.y;
                lift_v.y = wind_v.x;
            }
            else {
                lift_v.x = wind_v.y;
                lift_v.y = -wind_v.x;
            }

            //                printf("Lift vector: (%.3f, %.3f, %.3f)\n", lift_v.x, lift_v.y, lift_v.z);
            lift_v *= wind_speed * wind_speed * lift;

            //            float K_lift = 1 / 180.0;
            float K_lift = 1 / 250.0;
            lift_v *= K_lift;
            //                printf("Lift vector(2): (%.3f, %.3f, %.3f), lift, %.3f\n", lift_v.x, lift_v.y, lift_v.z, lift);
            accel_body += lift_v;

            DEBUGV("SimOcius: wind: %.2lf, lift: %.2lf, drag: %.2lf, attack: %.1lf\n", wind_speed, lift, wind_drag, attack_angle);
        }
    }
    //    printf("Current boat acceleration(%.2f, %.2f, %.2f)\n", accel_earth.x, accel_earth.y, accel_earth.z);

        // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;
    //}

    // new position vector
    position = velocity_ef * delta_time;

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
