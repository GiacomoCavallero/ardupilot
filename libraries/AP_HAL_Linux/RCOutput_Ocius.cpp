/*
 * RCOutput_Ocius.cpp
 *
 *  Created on: 29/08/2016
 *      Author: mmcgill
 */

#include "RCOutput_Ocius.h"
//#include <Rover.h>
#include "../../APMrover2/Rover.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <string>

#include <epos2_bridge.h>

#include <OC_NMEA2k/oc_filter.h>

#define BLUEBOTTLE_AIS_COVERT              ( 6 - 1)
#define BLUEBOTTLE_ODROID_PWR_CUT          ( 7 - 1)
#define BLUEBOTTLE_HYDRAULIC_SPD_CHANN     ( 8 - 1)
#define BLUEBOTTLE_MAST_CHANN              ( 9 - 1)
#define BLUEBOTTLE_SAIL_CHANN              (10 - 1)
#define BLUEBOTTLE_BATTERY_POWER_CHANN     (11 - 1)
#define BLUEBOTTLE_MAST_LOWER_CHANN        (12 - 1)
#define BLUEBOTTLE_MAST_RAISE_CHANN        (13 - 1)
#define BLUEBOTTLE_WINCH_CHANN             (14 - 1)

//#define BLUEBOTTLE_MAST_RELAY_DURATION     7500
//#define BLUEBOTTLE_MAST_RELAY_DELAY        500

#define BLUEBOTTLE_THREAD_PERIOD_WAIT      200000
#define BLUEBOTTLE_MOTOR_OFFSET            (rover.g2.sailboat.sail_epos_zero)
#define BLUEBOTTLE_MOTOR_TICKS_PER_90      41000

#define WINCH_ENCODER_DEPLOY        (rover.g2.winch.encoder_out)
#define WINCH_ENCODER_RETRACT       (rover.g2.winch.encoder_in)
#define WINCH_ENCODER_RANGE   (WINCH_ENCODER_DEPLOY - WINCH_ENCODER_RETRACT)

/* this function is run by the second thread */
void* thread_init(void *thread_data) {
    RCOutput_Ocius* thread = (RCOutput_Ocius*)thread_data;

    thread->stinger_sail_comm_thread();
    return NULL;
}

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
RCOutput_Ocius::RCOutput_Ocius(uint8_t addr, bool external_clock, uint8_t channel_offset,
                                int16_t oe_pin_number) : RCOutput_Ocius_Parent(addr, external_clock, channel_offset, oe_pin_number),
#else
RCOutput_Ocius::RCOutput_Ocius(uint8_t chip, uint8_t channel_base, uint8_t channel_count) : RCOutput_Ocius_Parent(chip, channel_base, channel_count),
#endif
    imu_filt(&(rover.g2.sailboat.tilt_filt))
{
    pwm_last = new uint16_t[_channel_count];
    pwm_status = new AP_HAL::ServoStatus[_channel_count];

    motor_enabled = new bool[_channel_count];

//    consecutive_failures = new int[_channel_count];
    last_move_attempt = new int[_channel_count];
    last_move_success = new int[_channel_count];
    last_move_time = new uint64_t[_channel_count];
    last_imu_update = 0;

    memset(pwm_last, 0, sizeof(uint16_t)*(unsigned int)channel_count);
    memset(pwm_status, 0, sizeof(AP_HAL::ServoStatus)*(unsigned int)channel_count);
    memset(motor_enabled, 0, sizeof(bool)*(unsigned int)channel_count);
//    memset(consecutive_failures, -1, sizeof(int)*(unsigned int)channel_count);
    memset(last_move_attempt, 0, sizeof(int)*(unsigned int)channel_count);
    memset(last_move_success, 0, sizeof(int)*(unsigned int)channel_count);
    memset(last_move_time, 0, sizeof(uint64_t)*(unsigned int)channel_count);

    closing = false;
    thread_flags = 0;
    // start thread for stinger_sail_comm_thread
    memset(&thread, 0, sizeof(thread));
    if (pthread_create(&thread, NULL, thread_init, this)) {
        return;
    }
}

RCOutput_Ocius::~RCOutput_Ocius() {
    closing = true;
    // TODO: Wait for stinger_sail_comm_thread to close
}

void RCOutput_Ocius::init() {
    RCOutput_Ocius_Parent::init();
    SRV_Channels::set_output_pwm_chan(0, 1500); // center the rudder
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, 1500);
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_BATTERY_POWER_CHANN, 1100);
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_ODROID_PWR_CUT, 1100);
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_AIS_COVERT, 1100);

    hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &RCOutput_Ocius::motor_status_check, void));
}

static uint64_t timeMastSignalStarted = 0;

#define mast_status     pwm_status[BLUEBOTTLE_MAST_CHANN]
#define sail_status     pwm_status[BLUEBOTTLE_SAIL_CHANN]
#define winch_status    pwm_status[BLUEBOTTLE_WINCH_CHANN]

void RCOutput_Ocius::write(uint8_t ch, uint16_t period_us) {
//    if (rover.g2.frame_class == FRAME_BLUEBOTTLE && ch == (rover.rcmap.roll()-1)) {
//        if (period_us < 1250) {
////            printf("Reducing rudder travel on Bruce(-).\n");
//            period_us = 1250;
//        } else if (period_us > 1850) {
////            printf("Reducing rudder travel on Bruce(+).\n");
//            period_us = 1850;
//        }
//    }

    RCOutput_Ocius_Parent::write(ch, period_us);

    if ((period_us == 0 && (rover.g2.frame_class != FRAME_BLUEBOTTLE || ch != BLUEBOTTLE_WINCH_CHANN)) ||
            rover.control_mode->mode_number() == Mode::Number::INITIALISING) {
        // This indicates no value set to pwm yet so ignore, or boat is still initialising
        return;
    } else if (period_us == 0  && pwm_last[ch] != 0) {
        printf("Recieved a PWM of 0 for the winch.\n");
        gcs().send_text(MAV_SEVERITY_NOTICE, "RCO_Ocius: Received a PWM of 0 for the winch.");
    }

    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        // redirect messages for Bluebottle
        if (ch == (rover.rcmap.throttle()-1)) {
            // No longer need to do anything, handled by the Torqeedo process
        } else if (ch == BLUEBOTTLE_MAST_CHANN) {
            // Set channels to raise/lower mast
            if (period_us == pwm_last[ch]) {
                // No change in PWM value, ignore
                //printf("RCO_Ocius: Repeated value. (%u)\n", period_us);
            } else if (period_us >= 1000 && period_us <= 1200) {
                // lower mast
                printf("RCO_Ocius: Pulling mast down. (%u)\n", period_us);
                // TODO: only activate relays if not currently active, shouldn't need to offset then
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, 1500);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1900);
                mast_status.moving = true;
                timeMastSignalStarted = AP_HAL::millis64();
                if (period_us == 1100) {
                    // We offset by 1, so that messages from GCS don't need to change
                    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_CHANN, 1101);
                }

            } else if (period_us >= 1800 && period_us <= 2000) {
                // raise mast
                printf("RCO_Ocius: Pushing mast up. (%u)\n", period_us);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, 1500);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1900);
                mast_status.moving = true;
                timeMastSignalStarted = AP_HAL::millis64();
                if (period_us == 1900) {
                    // We offset by 1, so that messages from GCS don't need to change
                    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_CHANN, 1899);
                }
            } else if (period_us == 0) {
                // Ignoring 0 value, need to locate the source
                printf("RCO_Ocius: Ignoring invalid 0 value.\n");
            } else {
                // stop all signals / unhome mast
                printf("RCO_Ocius: Killing mast hydraulics, unhoming motor. (%u)\n", period_us);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, 1500);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
                timeMastSignalStarted = 0;
                mast_status.homed = AP_HAL::SERVO_UNHOMED;
                mast_status.moving = false;
            }
        } else if (ch == BLUEBOTTLE_SAIL_CHANN) {
//            printf("Have a sail pwm to redirect to Stinger. (pwm: %u)\n", (uint32_t)period_us);
            // Send message to slew the sail, do nothing, messages handled in child thread
        }
    }
    pwm_last[ch] = period_us;
}

uint16_t RCOutput_Ocius::read(uint8_t ch) {
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        if (ch == (rover.rcmap.throttle()-1)) {
            return pwm_last[ch];
        }
    }
    return RCOutput_Ocius_Parent::read(ch);
}

void RCOutput_Ocius::home(uint8_t chan) {
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        switch (chan) {
        case BLUEBOTTLE_SAIL_CHANN:
        case BLUEBOTTLE_WINCH_CHANN:
            printf("RCOutput_Ocius::home_sail() - Homing sail on Stinger.\n");
            thread_flags = thread_flags | HomeSail;
            break;
        case BLUEBOTTLE_MAST_CHANN:
        {{
            if (mast_status.homed == AP_HAL::SERVO_HOMING) {
                // Wait for current homing to complete.
                return;
            }
    //        printf("RCOutput_Ocius::home_mast() - Homing mast on Stinger.\n");
    //        gcs().send_text(MAV_SEVERITY_NOTICE, "Homing mast on Stinger.");
            uint16_t home_pos = 1900;
            if (pwm_last[BLUEBOTTLE_MAST_CHANN] == 1900) {
                home_pos = 1899;
            }
            mast_status.homed = AP_HAL::SERVO_HOMING;
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_CHANN, home_pos);
        }}
            break;
        }
    }
}

AP_HAL::ServoStatus RCOutput_Ocius::read_status(uint8_t chan) {
    return pwm_status[chan];
}

void RCOutput_Ocius::read_status(AP_HAL::ServoStatus* status, uint8_t len) {
    for (uint32_t i = 0; i < len && i < _channel_count; ++i) {
        status[i] = pwm_status[i];
    }
}

#define GPIO_PIN 18

void RCOutput_Ocius::motor_status_check(void) {
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        // disable relay signal for mast
//		printf("Checking if mast signal to be disabled. (%u, %u)\n", timeMastSignalStarted, millis());

        // if the imu is configured and old, set mast_status.pwm to 0
        if (rover.g2.sailboat.tilt_imu != 0 && last_imu_update + 5000 < AP_HAL::millis64()) {
            // If the imu is set but we haven't updated in 5 seconds, we're not sure where the mast is
            mast_status.homed = AP_HAL::SERVO_UNHOMED;
//            mast_status.pwm = 0;
        }

        double hydraulic_run_time = rover.g2.sailboat.mast_time_up;
        if (pwm_last[BLUEBOTTLE_MAST_CHANN] < 1500) {
            hydraulic_run_time = rover.g2.sailboat.mast_time_down;
        }
        if (timeMastSignalStarted != 0 && rover.g2.sailboat.tilt_imu != 0 &&
                (abs((int)mast_status.pwm - (int)pwm_last[BLUEBOTTLE_MAST_CHANN]) <= rover.g2.sailboat.tilt_err)) {
            // If the hydraulics are running and the tilt imu is set, when we approach the position we cut the timer short.
            uint64_t shortTime = AP_HAL::millis64() + 1000;
            if (shortTime > hydraulic_run_time) {
                // avoiding negatives
                shortTime -= hydraulic_run_time;
            }

            if (shortTime < timeMastSignalStarted) {
                timeMastSignalStarted = shortTime;
            }
        }

        if (timeMastSignalStarted != 0 &&
                (AP_HAL::millis64() - timeMastSignalStarted) > hydraulic_run_time) {
            // Signal has passed desired time / kill it
            printf("RCO_Ocius: Time up. Mast homed.\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "Mast homed (%s)",
                    pwm_last[BLUEBOTTLE_MAST_CHANN] > 1800 ? "up":"down");
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, 1500);
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
            mast_status.homed = AP_HAL::SERVO_HOMED;
            mast_status.moving = false;
            if (rover.g2.sailboat.tilt_imu == 0) {
                // If the tilt IMU is 0, we just use the time to set the position.
                mast_status.pwm = pwm_last[BLUEBOTTLE_MAST_CHANN];
            }
            timeMastSignalStarted = 0;
            if (mast_status.pwm <= 1200) {
                // Mast is down, disable the sail motor.
                // This is performed in stinger_sail_comm_thread to avoid multiple threads talking to the EPOS
            }
        } else if (timeMastSignalStarted != 0 &&
                abs(sail_status.pwm - 1500) > rover.g2.sailboat.sail_stow_error &&
                pwm_last[BLUEBOTTLE_MAST_CHANN] < 1800) {
            // Sail is no longer centered, emergency abort lowering of the sail
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, 1500);
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
            mast_status.homed = AP_HAL::SERVO_UNHOMED;
            mast_status.moving = false;
            timeMastSignalStarted = 0;
            gcs().send_text(MAV_SEVERITY_ERROR, "Emergency STOP on mast servo. Sail not centered. (%u)", (uint32_t)sail_status.pwm);
        } else if (timeMastSignalStarted != 0) {
            uint32_t ramp_spd = 1900 - 1500;
            SRV_Channel* chan = SRV_Channels::srv_channel(BLUEBOTTLE_HYDRAULIC_SPD_CHANN);
            if (chan != nullptr) {
                ramp_spd = chan->get_output_max() - 1500;
            }
            if (pwm_last[BLUEBOTTLE_MAST_CHANN] < 1500) {
                // If driving mast down hydraulic speed is the smaller of (max - 1500) or (1500 - min)
                uint32_t down_spd = abs(1500 - chan->get_output_min());
                if (down_spd < ramp_spd)
                    ramp_spd = down_spd;
            }

            // Get the phase for the hydraulic speed
            double phase = 0;
            uint32_t relay_delay = rover.g2.sailboat.mast_time_delay;
            if (relay_delay <= 0) {
                // avoid divide by 0
                relay_delay = hydraulic_run_time / 2;
            }
            uint64_t signal_time = AP_HAL::millis64() - timeMastSignalStarted;
            if (hydraulic_run_time > 4 * relay_delay) {
                if (signal_time <= relay_delay || signal_time >= (hydraulic_run_time - relay_delay)) {
                    // We're in the delay period before or after we run.
                    signal_time = 0;
                    phase = 0;
                } else {
                    // adjust the signal and hydraulic runtime
                    signal_time -= relay_delay;
                    hydraulic_run_time -= 2 * relay_delay;
                }
            }
            if (signal_time < hydraulic_run_time / 2) {
                // 1st half of the signal pattern
                if (relay_delay > hydraulic_run_time / 2) {
                    // if delay is greater the 50% of run time, set it to 50% run time
                    relay_delay = hydraulic_run_time / 2;
                }
                if (signal_time < relay_delay) {
                    // we ramp up to max over the relay_delay time
                    phase = signal_time / relay_delay;
                } else {
                    // rest of the 1st half we run at full speed
                    phase = 1;
                }
            } else {
                // 2nd half of signal pattern
                phase = (hydraulic_run_time - signal_time) / (hydraulic_run_time/2);
//                phase = 1 - ((signal_time - (hydraulic_run_time/2)) /  (hydraulic_run_time/2));
            }

            ramp_spd = 1500 + (ramp_spd * phase);

            if (ramp_spd < 1500) {
                ramp_spd = 1500;
            } else if (ramp_spd > 1900) {
                ramp_spd = 1900;
            }
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, ramp_spd);
        }
    }
}

//void RCOutput_Ocius::motor_status_read(void){
////  DEBUGV("Reading motor status.\n");
//}

#define MOTOR_NAME (nodeid == 2?"Winch":"Sail")

static void RCOutput_Ocius_EmergencyHandler(uint8_t nodeid, uint16_t errCode) {
    if (nodeid == 1 || nodeid == 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "EPOS fault(0x%04x) detected on %s.", (uint32_t)errCode, MOTOR_NAME);
        uint16_t family;
        if (!getEPOSFamily(nodeid, &family, true)) {
            gcs().send_text(MAV_SEVERITY_WARNING, ":     %s", getErrorDescription(errCode, family));
        }
    }
}

bool bridge_initialised = false;
uint32_t last_initialise_fail_message = 0;
void RCOutput_Ocius::stinger_sail_comm_thread() {
    uint64_t last_bridge_try = 0;

    while (!closing) {
        usleep(BLUEBOTTLE_THREAD_PERIOD_WAIT);

        if (rover.g2.frame_class != FRAME_BLUEBOTTLE) {
            // Not talking to Stinger so sleep and try again.
            continue;
        }

        uint64_t now = AP_HAL::millis64();

        // initialise bridge
        if (!bridge_initialised) {
            std::string bridgeAddr = "10.42." + std::to_string(rover.g.sysid_this_mav) + ".104:20001";

            if (now - last_bridge_try < 1000) {
                // Wait 1s between retries.
                continue;
            }

            last_bridge_try = now;
            if (initializeBridge(0x04, bridgeAddr.c_str()) != 0) {
                // Initialisation failed.
                if (last_initialise_fail_message == 0 || (now - last_initialise_fail_message) > 10000) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: Unable to initialise bridge connection.");
                    printf("RCOut: Unable to initialise bridge connection.\n");
                    last_initialise_fail_message = now;
                }
                continue;
            }
            bridge_initialised = true;
            gcs().send_text(MAV_SEVERITY_INFO, "RCOut: Epos bridge initialised.");
            printf("RCOut: Epos bridge initialised.\n");
            last_initialise_fail_message = 0;
            sail_status.homed = AP_HAL::SERVO_UNHOMED;
            winch_status.homed = AP_HAL::SERVO_UNHOMED;

            sail_status.error_count = 0;
            winch_status.error_count = 0;

            setEmergencyHandler(&RCOutput_Ocius_EmergencyHandler);
        }

        if (motor_enabled[BLUEBOTTLE_SAIL_CHANN] && mast_status.homed == AP_HAL::SERVO_HOMED && mast_status.pwm <= 1200) {
            // mast is homed & down, sail enabled, so disable
            printf("Stinger: disabling sail motor.\n");
            if (!disableMotor(1)) {
                // Motor successfully disabled.
                motor_enabled[BLUEBOTTLE_SAIL_CHANN] = false;
            }
//            continue;
        }

        stinger_sail_update_epos(sail_status, BLUEBOTTLE_SAIL_CHANN, 1);
        if (rover.g2.winch.enable) {
            stinger_sail_update_epos(winch_status, BLUEBOTTLE_WINCH_CHANN, 2);
        }
    }
}


#define MAX_CONSEQ_FAILS     10
bool RCOutput_Ocius_stinger_epos_all_broken(uint8_t* nodeids, int node_count) {
    bool have_broken_reading = false;
    for (int i = 0; i < node_count; ++i) {
        if (nodeids[i] == 0) continue;
        if (getCANErrorCount(nodeids[i]) >= MAX_CONSEQ_FAILS) {
            have_broken_reading = true;
        } else {
            return false;
        }
    }
    return have_broken_reading;
}


void RCOutput_Ocius::stinger_sail_update_epos(AP_HAL::ServoStatus& motor, uint8_t ch, uint8_t nodeid) {
    if (!bridge_initialised) {
        return;
    }

    uint8_t homed;
    int32_t position = 0;
    float position_pwm_flt;

    // Get motor position
    if (readPosition(nodeid, &position)) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "EPOS: Servo %u(Node: %u) Read error. (%u)", (uint32_t)ch, (uint32_t)nodeid, __LINE__);
        motor.error_count++;
        motor._position_is_good = false;
        // Error reading motor position.
//        consecutive_failures[ch]++;
        if (getCANErrorCount(nodeid) >= MAX_CONSEQ_FAILS) {
//            getCANErrorCount(nodeid) = MAX_CONSEQ_FAILS;
            if (motor.homed != AP_HAL::SERVO_UNHOMED) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "RCOut: Consecutive fails on servo %u, unhoming motor", (uint32_t)ch);
//                printf("RCOut: Consecutive fails on servo %u, unhoming motor", (uint32_t)ch);
            }
            motor.homed = AP_HAL::SERVO_UNHOMED;

            uint8_t nodelist[2] =  {1,0};
            if (rover.g2.winch.enable && WINCH_ENCODER_DEPLOY != WINCH_ENCODER_RETRACT) {
                nodelist[1] = 2;
            }
            if (RCOutput_Ocius_stinger_epos_all_broken(nodelist, 2)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: Too many consecutive read failures. Resetting bridge connection.");
//                printf("RCOut: Too many consecutive read failures. Resetting bridge connection.\n");
                shutdownBridge();
                bridge_initialised = false;
                sail_status.homed = AP_HAL::SERVO_UNHOMED;
                winch_status.homed = AP_HAL::SERVO_UNHOMED;
//                memset(consecutive_failures, -1, sizeof(int)*(unsigned int)_channel_count);
            }
        }
        printf("RCOut: Read fail on channel %u, %d consecutive failures.\n", (uint32_t)ch, getCANErrorCount(nodeid));
//        goto THREAD_LOOP_SLEEP;
        return;
//    } else {
//        printf("Have motor %u position: %d\n", (uint32_t)nodeid, position);
//        fflush(stdout);
    }

//    if (consecutive_failures[ch] > 0) {
//        printf("RCOut: Successful read, clearing consecutive failures.\n");
//    }
//    consecutive_failures[ch] = 0;

    position_pwm_flt = 0;
    if (ch == BLUEBOTTLE_SAIL_CHANN) {
        position_pwm_flt = ((position - BLUEBOTTLE_MOTOR_OFFSET) * 400 / (float)BLUEBOTTLE_MOTOR_TICKS_PER_90) + 1500;  // Sail motor
    } else if (ch == BLUEBOTTLE_WINCH_CHANN) {
        if (WINCH_ENCODER_RANGE == 0) {
            position_pwm_flt = 0;
        } else {
            position_pwm_flt = 1100 + (((position - WINCH_ENCODER_RETRACT)/ (float)WINCH_ENCODER_RANGE) * 800);
        }
    }

    uint16_t motorFam = 0;
    getEPOSFamily(nodeid, &motorFam, false);
    switch(motorFam) {
    case FamilyEPOS2:
        motorFam = 2; break;
    case FamilyEPOS4:
        motorFam = 4; break;
    default:
        motorFam = 0; break;
    }

    motor.flag = motorFam;
    if (position_pwm_flt >= 1000 && position_pwm_flt <= 2000) {
        motor._position_is_good = true;
        motor._suspect_position_reads = 0;
    } else {
        motor.error_count++;
        motor._suspect_position_reads++;
        if (motor._suspect_position_reads <= 2) {
            // We ignore the 1st & 2nd possible bad reads, (due to a canfestival bug)
            // But we do skip the rest of the update.
            gcs().send_text(MAV_SEVERITY_DEBUG, "RCOut: Suspicious read(%u,raw:%d,pwm:%d) on %s.", motor._suspect_position_reads, position, (int)position_pwm_flt, MOTOR_NAME);
            return;
        } else if (motor._position_is_good) {
            // If the motor position previously was good, we disable to prevent damage
            gcs().send_text(MAV_SEVERITY_DEBUG, "RCOut: Suspicious read %u on %s. Disabling motor.", motor._suspect_position_reads, MOTOR_NAME);
            disableMotor(nodeid);
            motor_enabled[ch] = false;
            motor._position_is_good = false;
        } else if (motor._suspect_position_reads == 3) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "RCOut: Suspicious read(%u) on %s. Position already invalid.", motor._suspect_position_reads, MOTOR_NAME);
        }
    }
	//printf("EPOS %d is at position %d(%d)\n", nodeid, position, position_pwm);

    motor.moving = abs(motor.raw - position) > 5;  // if the raw reading has changes by 5 or more, the motor is considered as moving
    motor.raw = position;
    motor.pwm = (uint16_t)(::round(position_pwm_flt));

    if (!(motor.moving)) {
        uint64_t now = AP_HAL::millis64();
        // We check the homed state of the motor every 10 seconds or when the motor is unhomed
        if (motor.homed != AP_HAL::SERVO_HOMED || (now - motor._last_home_check > 10000)) {
            // Check motor is homed
            homed = false;
    //        if (motor.homed != AP_HAL::SERVO_HOMED)
    //            gcs().send_text(MAV_SEVERITY_WARNING, "Stinger: Checking home status of motor %d( %d )", ch, (int)motor.homed);
            if (isHomed(nodeid, &homed, NULL)) {
                motor.error_count++;
                // Error checking motor homed status
    //            goto THREAD_LOOP_SLEEP;
                return;
            }
            motor._last_home_check = now;
            //printf("Epos(%d) status is %s\n", nodeid, (homed?"homed":"UNhomed"));
            if (homed) {
                // Sail is homed
                if (motor.homed != AP_HAL::SERVO_HOMED) {
//                    printf("%s is homed.\n", MOTOR_NAME);
                    gcs().send_text(MAV_SEVERITY_NOTICE, "RCOut: %s is homed.", MOTOR_NAME);
                }
                motor.homed = AP_HAL::SERVO_HOMED;
                if (ch == BLUEBOTTLE_SAIL_CHANN) {
                    RCOutput_Ocius::thread_flags = RCOutput_Ocius::thread_flags & !HomeSail; // only for sail
                }
            } else {
                if (motor.homed == AP_HAL::SERVO_HOMED)
                    gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: %s has LOST its homing.", MOTOR_NAME);

                // Motor not homed
                motor.homed = AP_HAL::SERVO_UNHOMED;
            }
        }

        if (ch == BLUEBOTTLE_SAIL_CHANN && motor.homed == AP_HAL::SERVO_HOMED && motor_enabled[ch]) {
            int desiredPosition = (pwm_last[ch] - 1500) * BLUEBOTTLE_MOTOR_TICKS_PER_90 / 400 + BLUEBOTTLE_MOTOR_OFFSET;
            if (abs(motor.raw - desiredPosition) <= 100) {
                // Sail is not moving and close to its desired position, so we disable it
                disableMotor(nodeid);
                motor_enabled[ch] = false;
            }
        }
    }
 //            printf("Stinger: motor_enabled: %d, mast_flag: %d, mast_pos: %d\n",
//      motor_enabled, mast_status.flag, mast_status.position);

    if (motorFam == 4)
    {
        uint64_t now = AP_HAL::millis64();
        if (now - motor._last_device_update > 1000) {
            motor._last_device_update = now;

            uint64_t data;
            uint8_t size = sizeof(data);
            // read motor temp
            if (readNetworkEntry(nodeid, EPOS4_POWERSTAGETEMPERATURE_INDEX, 0x01, &size, &data)) {
                motor.error_count++;
                printf("Unable to read temperature on %s.\n", MOTOR_NAME);
            } else {
                motor.temperature = data*10;
            }

            // read power supply voltage
            if (readNetworkEntry(nodeid, EPOS4_POWERSUPPLYVOLTAGE_INDEX, 0x01, &size, &data)) {
                motor.error_count++;
                printf("Unable to read voltage on %s.\n", MOTOR_NAME);
            } else {
                motor.volts = data*10;
            }
        }
    }

    if (motor.homed != AP_HAL::SERVO_HOMED || (ch == BLUEBOTTLE_SAIL_CHANN && (thread_flags & HomeSail))) {
        if (motor.homed != AP_HAL::SERVO_HOMING) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: homing %s.", MOTOR_NAME);
//            printf("RCOut: homing %s.\n", MOTOR_NAME);
        }
        motor_enabled[ch] = true;
        homeMotor(nodeid);
        motor.homed = AP_HAL::SERVO_HOMING;
//        goto THREAD_LOOP_SLEEP;
        return;
    }

    if (!motor._position_is_good || motor._suspect_position_reads != 0) {
        // Last read position of motor is not good, not safe to move
        return;
    }

    if (!rover.arming.is_armed()) {
        if (ch == BLUEBOTTLE_SAIL_CHANN && pwm_last[ch] == 1500) {
            // Sail can be centered even when disarmed.
        } else {
            // Not armed so wait and check again
//            goto THREAD_LOOP_SLEEP;
            return;
        }
    }

    if (motor.homed != AP_HAL::SERVO_HOMED) {
        // Sail/Winch not homed so not safe to move.
//        goto THREAD_LOOP_SLEEP;
        return;
    }

    // PWMs from 1 .. 500 are used as an emergency stop
    if (ch == BLUEBOTTLE_WINCH_CHANN && pwm_last[ch] >= 1 && pwm_last[ch] <= 500) {
        // have emergency stop
        if (motor_enabled[ch]) {
            printf("RCOut: Emergency stop called on winch.\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "RCOut: Received emergency stop for winch.");
        }
        // disable motor power
        disableMotor(nodeid);
        // Set target position to current position
        moveToPosition(nodeid, position);
        motor_enabled[ch] = false;
        return;
    }

    if (ch == BLUEBOTTLE_WINCH_CHANN && pwm_last[ch] == 0) {
        // We're armed, but the winch hasn't been given a position, so can self deploy
        // We'll set its desired position to its current good position
        gcs().send_text(MAV_SEVERITY_NOTICE, "Sending STOP(%u) to EPOS on 1st arming.", (uint32_t)motor.pwm);
        SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_WINCH_CHANN, motor.pwm);
        return;
    }

    // Move sail/winch to desired position
    if (pwm_last[ch] >= 1075 && pwm_last[ch] <= 1925) {
        //printf("Comparing desired position with actual.\n");
        uint64_t now = AP_HAL::millis64();
        int desiredPosition = position;  // Default to prevent movement as desired is current position
        if (ch == BLUEBOTTLE_SAIL_CHANN) {
            if (mast_status.pwm != 0 && mast_status.pwm < 1800) {
                // Mast is down, not safe to move.
            } else if (mast_status.homed != AP_HAL::SERVO_HOMED &&
                    !(rover.g2.sailboat.tilt_imu != 0 && mast_status.pwm != 0)) {
                // Mast is not homed
            } else {
                desiredPosition = (pwm_last[ch] - 1500) * BLUEBOTTLE_MOTOR_TICKS_PER_90 / 400 + BLUEBOTTLE_MOTOR_OFFSET;
            }
            if (!rover.g2.sailboat.sail_is_safe() && desiredPosition != BLUEBOTTLE_MOTOR_OFFSET) {
                // if the sail is not safe it can only be centered, in preparation for stowing
                desiredPosition = position;
            }
        } else if (ch == BLUEBOTTLE_WINCH_CHANN) {
            desiredPosition = ((pwm_last[ch] - 1100) / 800.0) * WINCH_ENCODER_RANGE + WINCH_ENCODER_RETRACT;
//            printf("Desired winch position is %d.\n", desiredPosition);
        }
        if (abs(desiredPosition - position) > 100) {
            if (desiredPosition == last_move_attempt[ch] && desiredPosition == last_move_success[ch] &&
                    (now - last_move_time[ch]) < 5000) {
                // Wait 5 seconds after a successful send before retrying.
            } else {
                if(enableMotor(nodeid)) {
                    motor.error_count++;
                    // Error ensuring motor is enabled.
//                    goto THREAD_LOOP_SLEEP;
                    return;
                }

                if (enablePositionProfileMode(nodeid)) {
                    // Error ensuring motor is in position profile mode
//                    goto THREAD_LOOP_SLEEP;
                    return;
                }

                motor_enabled[ch] = true;
//                printf("RCOut: Moving %s to position %d.\n", MOTOR_NAME, desiredPosition);
                last_move_attempt[ch] = desiredPosition;
                if (!moveToPosition(nodeid, desiredPosition)) {
                    last_move_success[ch] = desiredPosition;
                    last_move_time[ch] = now;
                } else {
                    motor.error_count++;
                }
            }
        }
    }
}

static uint64_t next_device_status = 0;
void RCOutput_Ocius::send_epos_status(uint8_t chan) {
    uint16_t epos1_family = 0, epos2_family = 0;  // TODO: these may need conversion before being cast to a uint8_t in the message
    if (getEPOSFamily(1, &epos1_family, true)) epos1_family = 0;
    if (getEPOSFamily(2, &epos2_family, true)) epos2_family = 0;
    mavlink_msg_epos_status_send((mavlink_channel_t)chan,
            epos1_family, sail_status.raw, sail_status.pwm, last_move_attempt[BLUEBOTTLE_SAIL_CHANN], pwm_last[BLUEBOTTLE_SAIL_CHANN],
            epos2_family, winch_status.raw, winch_status.pwm, last_move_attempt[BLUEBOTTLE_WINCH_CHANN], pwm_last[BLUEBOTTLE_WINCH_CHANN],
            0, 0, 0, 0, 0, // epos 3
            0, 0, 0, 0, 0, // epos 4
            0, 0, 0, 0, 0, // epos 5
            0, 0, 0, 0, 0, // epos 6
            0, 0, 0, 0, 0, // epos 7
            0, 0, 0, 0, 0, // epos 8
            sail_status.error_count, winch_status.error_count, 0, 0, 0, 0, 0, 0); // error counts

    uint64_t now = AP_HAL::millis64();

    // Only for bluebottles, send device status for the CAN & epos servos
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE && (next_device_status == 0 || now >= next_device_status)) {
        next_device_status += 15000;
        if (next_device_status < now)
            next_device_status = now + 15000;

        // send CAN Bus status
        DEVICE_STATUS dev_status = DEVICE_STATUS_HEALTHY;
        std::string dev_report = "";

        if (!bridge_initialised) {
            dev_status = DEVICE_STATUS_FAULT;
            dev_report = "No connection to can-ethernet bridge.";
        }
        mavlink_msg_oc_device_status_send((mavlink_channel_t)chan, 20, dev_status, 0, 0, 0, 0);
        mavlink_msg_oc_device_ext_status_send((mavlink_channel_t)chan, 20, 0, 0, 0, 0, 0, 0, 0, dev_report.c_str());

        if (rover.g2.sailboat.enable) {
            // send sail status
            uint16_t motor_fam = 0;
            if (getEPOSFamily(1, &motor_fam, true) != 0 || motor_fam == 0) {
                dev_status = DEVICE_STATUS_FAULT;
                dev_report = "Not connected to EPOS.";
            } else if (!sail_status._position_is_good) {
                dev_status = DEVICE_STATUS_FAULT;
                dev_report = "Sail position is not valid.";
            } else  if (getCANErrorCount(1)) {
                dev_status = DEVICE_STATUS_WARN;
                dev_report = "Sail has read errors.";
            } else if (sail_status.homed != AP_HAL::SERVO_HOMED) {
                dev_status = DEVICE_STATUS_WARN;
                dev_report = "Sail isn't homed.";
            } else {
                dev_status = DEVICE_STATUS_HEALTHY;
                dev_report = "";
            }

            mavlink_msg_oc_device_status_send((mavlink_channel_t)chan, 21, dev_status, sail_status.temperature * 0.01, 0, 0, 0);
            mavlink_msg_oc_device_ext_status_send((mavlink_channel_t)chan, 21, 0, 0, sail_status.volts * 0.01, 0, 0, 0, 0, dev_report.c_str());
        }

        if (rover.g2.winch.enable) {
            // send winch status
            uint16_t motor_fam = 0;
            if (getEPOSFamily(2, &motor_fam, true) != 0 || motor_fam == 0) {
                dev_status = DEVICE_STATUS_FAULT;
                dev_report = "Not connected to EPOS.";
            } else if (!winch_status._position_is_good) {
                dev_status = DEVICE_STATUS_FAULT;
                dev_report = "Winch position is not valid.";
            } else  if (getCANErrorCount(1)) {
                dev_status = DEVICE_STATUS_WARN;
                dev_report = "Winch has read errors.";
            } else if (winch_status.homed != AP_HAL::SERVO_HOMED) {
                dev_status = DEVICE_STATUS_WARN;
                dev_report = "Winch isn't homed.";
            } else {
                dev_status = DEVICE_STATUS_HEALTHY;
                dev_report = "";
            }

            mavlink_msg_oc_device_status_send((mavlink_channel_t)chan, 22, dev_status, winch_status.temperature * 0.01, 0, 0, 0);
            mavlink_msg_oc_device_ext_status_send((mavlink_channel_t)chan, 22, 0, 0, winch_status.volts * 0.01, 0, 0, 0, 0, dev_report.c_str());
        }
    }
}

void RCOutput_Ocius::updateMastIMU(int16_t xacc, int16_t yacc, int16_t zacc) {
    // get acc from onboard imu
    Vector3f boat_accel = AP::ins().get_accel();

    // calc mast angle
    Vector3f mast_accel(-xacc, -yacc, zacc);   // The IMU on the sail has a 180 degree roll, so we correct it here

    if (boat_accel.length_squared() == 0 || mast_accel.length_squared() == 0) {
        // Need unit vectors so have to skip.
        //printf("[%s] Zero vector on IMU.\n", __func__);
        return;
    }

    // Filter out anomalous readings, can be due to sensor failure, or acceleration due to impact, or exceptional vehicle states
    if (mast_accel.x > 500) {
        // Either the vessel is upside down, or there is a lot of acceleration in the X direction, or the sensor reading is false
        //printf("[%s] mast_accel.x is too high.\n", __func__);
        return;
    } else if (fabs(mast_accel.length() - 1000) > 500 || fabs(boat_accel.length() - 10) > 5) {
        // 1G acceleration should be ~1000(mast)/10(boat), readings too small/large indicate bad readings or possible impacts, which could alter the pitch calculations
        //printf("[%s] Acceleration is not close to 1G.\n", __func__);
        return;
    }

    last_imu_update = AP_HAL::millis64();
//printf("RAW boat: (%f, %f, %f), mast: (%f, %f, %f)\n", boat_accel.x, boat_accel.y, boat_accel.z, mast_accel.x, mast_accel.y, mast_accel.z);
    boat_accel.normalize();
    mast_accel.normalize();
//printf("NORM boat: (%f, %f, %f), mast: (%f, %f, %f)\n", boat_accel.x, boat_accel.y, boat_accel.z, mast_accel.x, mast_accel.y, mast_accel.z);

    if (fabs(boat_accel.y) > 0.95 || fabs(mast_accel.y) > 0.95) {
        // If we roll too far, then the readings can be swamped by noise, so we skip the update hoping to roll back upright
        //printf("[%s] Boat is heavily rolled.\n", __func__);
        return;
    }

    float boat_pitch = degrees(atan2(-boat_accel.x, -boat_accel.z));
    float mast_pitch = degrees(atan2(-mast_accel.x, -mast_accel.z));

    float angle = wrap_180(mast_pitch - boat_pitch);
    // FIXME: Adding 95 PWM to calibrate the new sensor, this should be parameterised
    int pwm = (int)(angle*800/90+1100);
//printf("PITCH boat: %f, mast: %f, ANGLE: %f, PWM: %d \n", boat_pitch, mast_pitch, angle, pwm);
    int pwm_filt = imu_filt.filterPoint(pwm);

    // When within 100 PWM of 1100/1900 we latch to the value closest to the desired value
    if (pwm_filt >= 1800 && pwm_filt <= 2000) {
        if (abs(pwm_filt - 1900) < abs(((int)mast_status.pwm) - 1900)) {
            mast_status.pwm = pwm_filt;
        }
    } else if (pwm_filt >= 1000 && pwm_filt <= 1200) {
        if (abs(pwm_filt - 1100) < abs(((int)mast_status.pwm) - 1100)) {
            mast_status.pwm = pwm_filt;
        }
    } else {
        mast_status.pwm = pwm_filt;
    }
    mast_status.homed = AP_HAL::SERVO_HOMED;
}

void RCOutput_Ocius::updateServoPosition(uint8_t chan, uint16_t pwm) {
    if (chan >= _channel_count) return;

    pwm_status[chan].pwm = pwm;
}
