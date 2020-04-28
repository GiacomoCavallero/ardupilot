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
#include <../APMrover2/Rover.h>
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
        have_reading = nmea2k_sensors.primary_gps.last_update != 0 && nmea2k_sensors.compass.last_update != 0;
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

    if (nmea2k_sensors.primary_gps.last_update == 0) {
        // Need a GPS position 1st.
        return;
    }
    Location location = nmea2k_sensors.primary_gps.location;

    if (last_plublished_ms >= nmea2k_sensors.compass.last_update) {
        // Only publish new readings.
        return;
    } else if (fabs(nmea2k_sensors.compass.pitch) >= radians(45)) {
        // Stop publishing compass readings when the comms mast is down, otherwise it will just report 0 degrees
        return;
    }
    last_plublished_ms = nmea2k_sensors.compass.last_update;

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

    float roll  = nmea2k_sensors.compass.roll,
          pitch = nmea2k_sensors.compass.pitch,
          //yaw   = nmea2k_sensors.compass.yaw,
          heading = nmea2k_sensors.compass.heading;

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
    heading += rover.g2.magnetic_offset;
#endif

    Matrix3f dcm;
    dcm.from_euler(roll, pitch, wrap_PI(radians(heading)));
    
    // Rotate into body frame
    Vector3f mag_bf = dcm.transposed() * mag_ef;
    publish_raw_field(mag_bf, compass_instance);
    publish_filtered_field(mag_bf, compass_instance);
}
