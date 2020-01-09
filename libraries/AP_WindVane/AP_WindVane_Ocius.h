#pragma once

#include "AP_WindVane_Backend.h"

class AP_WindVane_Ocius : public AP_WindVane_Backend
{
public:
    // constructor
    AP_WindVane_Ocius(AP_WindVane &frontend);
    void update_direction() override;
    void update_speed() override;
};
