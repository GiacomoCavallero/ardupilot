#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#define SITL_NUM_COMPASSES 3

class AP_Compass_Ocius : public AP_Compass_Backend {
public:
    AP_Compass_Ocius();

    void read(void) override;
    bool init(void);


    // detect the sensor
    static AP_Compass_Backend *probe();

protected:

    uint8_t compass_instance;
    uint64_t last_plublished_ms;
};
