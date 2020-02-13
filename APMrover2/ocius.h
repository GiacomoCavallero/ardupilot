/*
 * nmea2k.h
 *
 *  Created on: 30 Oct. 2019
 *      Author: mmcgill
 */

#pragma once

#define KNOTS_PER_METRE     1.94384

#define CLIP_180(A)     (A < -180? A + 360: A > 180? A - 360: A)
#define CLIP_360(A)     (A < 0? A + 360: A > 360? A - 360: A)

enum SAIL_MODE {
    MOTOR_ONLY=0,
    MOTOR_SAIL=1,
    SAIL_ONLY=2,
    WAVE_POWER=3,
    MOTOR_SOLAR=5
};

#define SAIL_MIN_WIND   5   // Knots

enum WIND_STRENGTH {
    WIND_LOW = 1,
    WIND_FAIR = 2,
    WIND_STRONG = 4,
    WIND_HIGH = 8,

    WIND_GOOD = 6,
};

//enum MOTOR_HOMING_STATUS {
//    MOTOR_UNHOMED   = 0,
//    MOTOR_HOMED     = 1,
//    MOTOR_HOMING    = 2,
//};

enum SAIL_HOLD_MODE {
    HOLD_DRIFT      = 0,
    HOLD_ACTIVE     = 1,
    HOLD_FIGURE8    = 2,
};

//enum VESSEL_TYPE {
//    VESSEL_UNKNOWN      = 0,
//    VESSEL_NEMO         = 1,
//    VESSEL_STINGER      = 2,
//};

enum SAIL_FLAGS {
    SAIL_NO_FLAGS = 0,
    SAIL_JIBE_ONLY = 1,
//    SAIL_IGNORE_XTRACK = 2,     // Replaced with NAVL1_IGN_XTRACK
    SAIL_HEADING_TO_WP = 4,
};
