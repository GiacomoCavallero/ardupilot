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
                                int16_t oe_pin_number) : RCOutput_Ocius_Parent(addr, external_clock, channel_offset, oe_pin_number) {
#else
RCOutput_Ocius::RCOutput_Ocius(uint8_t chip, uint8_t channel_base, uint8_t channel_count) : RCOutput_Ocius_Parent(chip, channel_base, channel_count) {
#endif
    pwm_last = new uint16_t[_channel_count];
    pwm_status = new AP_HAL::ServoStatus[_channel_count];

    motor_enabled = new bool[_channel_count];

    consecutive_failures = new int[_channel_count];
    last_move_attempt = new int[_channel_count];
    last_move_success = new int[_channel_count];
    last_move_time = new int[_channel_count];

    memset(pwm_last, 0, sizeof(uint16_t)*(unsigned int)channel_count);
    memset(pwm_status, 0, sizeof(AP_HAL::ServoStatus)*(unsigned int)channel_count);
    memset(motor_enabled, 0, sizeof(bool)*(unsigned int)channel_count);
    memset(consecutive_failures, -1, sizeof(int)*(unsigned int)channel_count);
    memset(last_move_attempt, 0, sizeof(int)*(unsigned int)channel_count);
    memset(last_move_success, 0, sizeof(int)*(unsigned int)channel_count);
    memset(last_move_time, 0, sizeof(int)*(unsigned int)channel_count);

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

static uint32_t timeMastSignalStarted = 0;

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
                timeMastSignalStarted = millis();
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
                timeMastSignalStarted = millis();
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

        double hydraulic_run_time = rover.g2.sailboat.mast_time_up;
        if (pwm_last[BLUEBOTTLE_MAST_CHANN] < 1500) {
            hydraulic_run_time = rover.g2.sailboat.mast_time_down;
        }

        if (timeMastSignalStarted != 0 &&
                (millis() - timeMastSignalStarted) > hydraulic_run_time) {
            // Signal has passed desired time / kill it
            printf("RCO_Ocius: Time up. Mast homed.\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "Mast homed (%s)",
                    pwm_last[BLUEBOTTLE_MAST_CHANN] > 1800 ? "up":"down");
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_HYDRAULIC_SPD_CHANN, 1500);
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
            mast_status.homed = AP_HAL::SERVO_HOMED;
            mast_status.moving = false;
            mast_status.pwm = pwm_last[BLUEBOTTLE_MAST_CHANN];
            timeMastSignalStarted = 0;
            if (mast_status.pwm <= 1200) {
                // Mast is down, disable the sail motor.
                // This is performed in stinger_sail_comm_thread to avoid multiple threads talking to the EPOS
            }
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
            uint32_t signal_time = millis() - timeMastSignalStarted;
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
        gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: EPOS fault(0x%04x) detected on %s.", (uint32_t)errCode, MOTOR_NAME);
        uint16_t family;
        if (!getEPOSFamily(nodeid, &family)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RCOut:     %s", getErrorDescription(errCode, family));
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
        if (rover.g2.winch.enable && WINCH_ENCODER_DEPLOY != WINCH_ENCODER_RETRACT) {
            stinger_sail_update_epos(winch_status, BLUEBOTTLE_WINCH_CHANN, 2);
        }
    }
}


#define MAX_CONSEQ_FAILS     10
bool RCOutput_Ocius_stinger_epos_all_broken(int* consecutive_failures, int channel_count) {
    bool have_broken_reading = false;
    for (int i = 0; i < channel_count; ++i) {
        if (consecutive_failures[i] >= MAX_CONSEQ_FAILS) {
            have_broken_reading = true;
        } else if (consecutive_failures[i] >= 0) {
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
    int32_t position;
    uint16_t position_pwm;


    // Get motor position
    if (readPosition(nodeid, &position)) {
        motor._position_is_good = false;
        // Error reading motor position.
        consecutive_failures[ch]++;
        if (consecutive_failures[ch] >= MAX_CONSEQ_FAILS) {
            consecutive_failures[ch] = MAX_CONSEQ_FAILS;
            if (motor.homed != AP_HAL::SERVO_UNHOMED) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "RCOut: Consecutive fails on servo %u, unhoming motor", (uint32_t)ch);
//                printf("RCOut: Consecutive fails on servo %u, unhoming motor", (uint32_t)ch);
            }
            motor.homed = AP_HAL::SERVO_UNHOMED;

            if (RCOutput_Ocius_stinger_epos_all_broken(consecutive_failures, (int)_channel_count)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: Too many consecutive read failures. Resetting bridge connection.");
//                printf("RCOut: Too many consecutive read failures. Resetting bridge connection.\n");
                shutdownBridge();
                bridge_initialised = false;
                memset(consecutive_failures, -1, sizeof(int)*(unsigned int)_channel_count);
            }
        }
        printf("RCOut: Read fail on channel %u, %d consequetive failures.\n", (uint32_t)ch, consecutive_failures[ch]);
//        goto THREAD_LOOP_SLEEP;
        return;
//    } else {
//        printf("Have motor %u position: %d\n", (uint32_t)nodeid, position);
//        fflush(stdout);
    }

//    if (consecutive_failures[ch] > 0) {
//        printf("RCOut: Successful read, clearing consecutive failures.\n");
//    }
    consecutive_failures[ch] = 0;

    position_pwm = 0;
    if (ch == BLUEBOTTLE_SAIL_CHANN) {
        position_pwm = ((position - BLUEBOTTLE_MOTOR_OFFSET) * 400 / BLUEBOTTLE_MOTOR_TICKS_PER_90) + 1500;  // Sail motor
    } else if (ch == BLUEBOTTLE_WINCH_CHANN) {
        position_pwm = 1100 + (((position - WINCH_ENCODER_RETRACT)/ (float)WINCH_ENCODER_RANGE) * 800);
    }
    motor.moving = abs(motor.raw - position) > 5;  // if the raw reading has changes by 5 or more, the motor is considered as moving
    motor.raw = position;
    motor.pwm = position_pwm;
    if (motor.pwm >= 1000 && motor.pwm <= 2000) {
        motor._position_is_good = true;
    } else {
        motor._position_is_good = false;
    }
	//printf("EPOS %d is at position %d(%d)\n", nodeid, position, position_pwm);

    if (!(motor.moving)) {
        uint64_t now = AP_HAL::millis64();
        // We check the homed state of the motor every 10 seconds or when the motor is unhomed
        if (motor.homed != AP_HAL::SERVO_HOMED || (now - motor._last_home_check > 10000)) {
            // Check motor is homed
            homed = false;
    //        if (motor.homed != AP_HAL::SERVO_HOMED)
    //            gcs().send_text(MAV_SEVERITY_WARNING, "Stinger: Checking home status of motor %d( %d )", ch, (int)motor.homed);
            if (isHomed(nodeid, &homed, NULL)) {
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
                RCOutput_Ocius::thread_flags = RCOutput_Ocius::thread_flags & !HomeSail; // FIXME: only for sail
            } else {
                if (motor.homed == AP_HAL::SERVO_HOMED)
                    gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: %s has LOST its homing.", MOTOR_NAME);

                // Motor not homed
                motor.homed = AP_HAL::SERVO_UNHOMED;
            }
        }
    }
 //            printf("Stinger: motor_enabled: %d, mast_flag: %d, mast_pos: %d\n",
//      motor_enabled, mast_status.flag, mast_status.position);

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

    if (!motor._position_is_good) {
        // Last read position of motor is not good, not safe to move
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
    if (pwm_last[ch] >= 1100 && pwm_last[ch] <= 1900) {
        //printf("Comparing desired position with actual.\n");
        int now = millis();
        int desiredPosition = position;  // Default to prevent movement as desired is current position
        if (ch == BLUEBOTTLE_SAIL_CHANN) {
            if (mast_status.homed != AP_HAL::SERVO_HOMED || mast_status.pwm < 1800) {
                // If mast not homed or not up don't move sail
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
                }
            }
        }
    }
}
