#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <pthread.h>
//#include "include/mavlink/v2.0/mavlink_types.h"
#include <OC_NMEA2k/oc_filter.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
#include "RCOutput_PCA9685.h"
typedef Linux::RCOutput_PCA9685 RCOutput_Ocius_Parent;
#else
#include "RCOutput_Sysfs.h"
typedef Linux::RCOutput_Sysfs RCOutput_Ocius_Parent;
#endif

class RCOutput_Ocius : public RCOutput_Ocius_Parent {
protected:
    uint16_t* pwm_last;
    AP_HAL::ServoStatus *pwm_status;

    bool* motor_enabled;
//    int* consecutive_failures;
    int* last_move_attempt;
    int* last_move_success;
    uint64_t* last_move_time;
    uint64_t last_imu_update;
    FiltExp<float> imu_filt;

    char linebuf[128];
    uint8_t linebuf_len = 0;

    enum ThreadFlags {
        None = 0,
        HomeMast = 1,
        HomeSail = 2
    };
    bool closing;
    pthread_t thread;
public:
    uint8_t thread_flags;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
    RCOutput_Ocius(uint8_t addr, bool external_clock, uint8_t channel_offset,
                              int16_t oe_pin_number);
#else
    RCOutput_Ocius(uint8_t chip, uint8_t channel_base, uint8_t channel_count);
#endif
    ~RCOutput_Ocius();

    static RCOutput_Ocius *from(AP_HAL::RCOutput *rcoutput)
    {
        return static_cast<RCOutput_Ocius *>(rcoutput);
    }

    void init() override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
//    uint16_t read_pos(uint8_t ch);

    /* On servos that have a potentiometer or other sensor, this returns
     * the PWM of the current position. */
    AP_HAL::ServoStatus read_status(uint8_t chan) override;
    void read_status(AP_HAL::ServoStatus* status, uint8_t len) override;


    /*
     * Home a single servo.
     * cork() has been called before.
     */
    void     home(uint8_t chan) override;
//    void home_mast();
//    void home_sail();
    void motor_status_check(void);
//    void motor_status_read(void);

    void stinger_sail_comm_thread();
    void stinger_check_mast_signal();
    void stinger_sail_update_epos(AP_HAL::ServoStatus& motor, uint8_t ch, uint8_t nodeid);
    void send_epos_status(uint8_t chan);
    void updateMastIMU(int16_t xacc, int16_t yacc, int16_t zacc);
    void updateServoPosition(uint8_t chan, uint16_t pwm);
};
