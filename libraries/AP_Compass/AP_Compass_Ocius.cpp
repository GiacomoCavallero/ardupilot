#include "AP_Compass_Ocius.h"
#include <OC_NMEA2k/OC_NMEA2k.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>
#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
#include <../APMrover2/Rover.h>
#endif

AP_Compass_Ocius::AP_Compass_Ocius() : 
    AP_Compass_Backend(),
    compass_instance(0), last_plublished_ms(0)
{
    _compass._setup_earth_field();
}

AP_Compass_Backend *AP_Compass_Ocius::probe() {
    AP_Compass_Ocius* driver = nullptr;
	
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    if (rover.get_frame_class() == FRAME_BLUEBOTTLE || rover.get_frame_class() == FRAME_WAMV) {
        driver = new AP_Compass_Ocius();
        driver->init();
    }
#endif

#endif

    return driver;
}

bool AP_Compass_Ocius::init(void) {
    compass_instance = register_compass();
    set_dev_id(compass_instance, DEVTYPE_OCIUS);
    nmea2k_sensors.init();

    uint64_t tstart = AP_HAL::millis64(), tdiff = 0;
    bool have_reading = false;
    while (!have_reading && tdiff <= 1000) {
        usleep(1000);
        //nmea2k_sensors.read();
        NMEA2K::GPS* gps = nmea2k_sensors.get_active_gps();
        have_reading = (gps != NULL) && nmea2k_sensors.primary_compass.last_update != 0;
        tdiff = AP_HAL::millis64() - tstart;
    }
    read();
    return have_reading;
}

void AP_Compass_Ocius::read()
{
    /*
     * A compass measurement is expected to pass through the following functions:
     * 1. rotate_field - this rotates the measurement in-place from sensor frame
     *      to body frame
     * 2. publish_raw_field - this provides an uncorrected point-sample for
     *      calibration libraries
     * 3. correct_field - this corrects the measurement in-place for hard iron,
     *      soft iron, motor interference, and non-orthagonality errors
     * 4. publish_filtered_field - legacy filtered magnetic field
     *
     * All those functions expect the mag field to be in milligauss.
     */

    Location location = nmea2k_sensors.get_location();

    // failover to secondary compass
    NMEA2K::Compass *compass = NULL;
    if (nmea2k_sensors.primary_compass.last_update > last_plublished_ms &&
                fabs(nmea2k_sensors.primary_compass.pitch) < radians(45)) {
        // Primary compass is not pitched forward/back and has more recent readings
        // When the comms mast is lowered the AIRMAR compass stops reporting useful values
        compass = &nmea2k_sensors.primary_compass;
    } else if (nmea2k_sensors.secondary_compass.last_update > last_plublished_ms &&
            fabs(nmea2k_sensors.secondary_compass.pitch) < radians(45)) {
        // Secondary compass is not pitched forward/back and has more recent readings
        compass = &nmea2k_sensors.secondary_compass;
    }

    if (compass == NULL) {
        // No compass found with fresh valid data.
        return;
    }
    last_plublished_ms = compass->last_update;

    // get the magnetic field intensity and orientation
    float intensity;
    float declination;
    float inclination;
    AP_Declination::get_mag_field_ef(location.lat * 1e-7f, location.lng * 1e-7f, intensity, declination, inclination);

    // create a field vector and rotate to the required orientation
    Vector3f mag_ef(1e3f * intensity, 0.0f, 0.0f);
    Matrix3f R;
//    printf("mag intensity: %.3f, inclination: %.3f, devlination: %.3f\n",
//           intensity, inclination, declination);
    R.from_euler(0.0f, -ToRad(inclination), ToRad(declination));
    mag_ef = R * mag_ef;

    float roll  = compass->roll,
          pitch = compass->pitch,
          //yaw   = compass->yaw,
          heading = compass->heading;

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    if (rover.g2.nmea2k.use_filtered) {
        heading = compass->heading_filt;
    }
    heading += rover.g2.magnetic_offset;
#endif

    Matrix3f dcm;
    dcm.from_euler(roll, pitch, wrap_PI(radians(heading)));
    
    // Rotate into body frame
    Vector3f mag_bf = dcm.transposed() * mag_ef;

    rotate_field(mag_bf, compass_instance);
    publish_raw_field(mag_bf, compass_instance);
    correct_field(mag_bf, compass_instance);
    publish_filtered_field(mag_bf, compass_instance);
}
