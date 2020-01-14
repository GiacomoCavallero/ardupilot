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

AP_Compass_Ocius::AP_Compass_Ocius() : compass_instance(0), last_plublished_ms(0) {
}

AP_Compass_Backend *AP_Compass_Ocius::probe() {
    AP_Compass_Ocius* driver = nullptr;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
#include <../APMrover2/Rover.h>
    if (rover.get_frame_type() == FRAME_BLUEBOTTLE || rover.get_frame_type() == FRAME_WAMV) {
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

    return true;
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
    Location location;

    if (!AP::ahrs().get_position(location)) {
        // Need a GPS position 1st.
        return;
    }

    if (last_plublished_ms >= nmea2k_sensors.compass.last_update) {
        // Only publish new readings.
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
    R.from_euler(0.0f, -ToRad(inclination), ToRad(declination));
    mag_ef = R * mag_ef;

    Matrix3f dcm;
    dcm.from_euler(nmea2k_sensors.compass.roll, nmea2k_sensors.compass.pitch, nmea2k_sensors.compass.yaw);

    // Rotate into body frame
    Vector3f mag_bf = dcm.transposed() * mag_ef;
    publish_raw_field(mag_bf, compass_instance);
    publish_filtered_field(mag_bf, compass_instance);
}
