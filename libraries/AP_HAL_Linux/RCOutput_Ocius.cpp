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

#define BLUEBOTTLE_MAST_CHANN              ( 9 - 1)
#define BLUEBOTTLE_SAIL_CHANN              (10 - 1)
#define BLUEBOTTLE_BATTERY_POWER_CHANN     (11 - 1)
#define BLUEBOTTLE_MAST_LOWER_CHANN        (12 - 1)
#define BLUEBOTTLE_MAST_RAISE_CHANN        (13 - 1)
#define BLUEBOTTLE_WINCH_CHANN             (14 - 1)

#define BLUEBOTTLE_MAST_RELAY_DURATION     7500

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
    // TODO: Add any constructor code here

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
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_BATTERY_POWER_CHANN, 1100);
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
}

static uint32_t timeMastSignalStarted = 0;

#define mast_status     pwm_status[BLUEBOTTLE_MAST_CHANN]
#define sail_status     pwm_status[BLUEBOTTLE_SAIL_CHANN]
#define winch_status    pwm_status[BLUEBOTTLE_WINCH_CHANN]

void RCOutput_Ocius::write(uint8_t ch, uint16_t period_us) {
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE && ch == (rover.rcmap.roll()-1)) {
        if (period_us < 1250) {
//            printf("Reducing rudder travel on Bruce(-).\n");
            period_us = 1250;
        } else if (period_us > 1850) {
//            printf("Reducing rudder travel on Bruce(+).\n");
            period_us = 1850;
        }
    }

    RCOutput_Ocius_Parent::write(ch, period_us);

    if ((period_us == 0 && (rover.g2.frame_class != FRAME_BLUEBOTTLE || ch != BLUEBOTTLE_WINCH_CHANN)) ||
            rover.control_mode->mode_number() == Mode::Number::INITIALISING) {
        // This indicates no value set to pwm yet so ignore, or boat is still initialising
        return;
    } else if (period_us == 0  && pwm_last[ch] != 0) {
        printf("Recieved a PWM of 0 for the winch.\n");
        gcs().send_text(MAV_SEVERITY_NOTICE, "RCO_Ocius: Recieved a PWM of 0 for the winch.\n");
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
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
                SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1900);
                mast_status.moving = true;
                timeMastSignalStarted = millis();
                if (period_us == 1100) {
                    // We offset by 1, so that messages from GCS don't need to change
                    SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_CHANN, 1101);
                }
                // TODO Disable sail motor

            } else if (period_us >= 1800 && period_us <= 2000) {
                // raise mast
                printf("RCO_Ocius: Pushing mast up. (%u)\n", period_us);
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

//uint16_t RCOutput_Ocius::read_pos(uint8_t ch) {
//    // TODO: RCOutput_Ocius::read_pos(uint8_t ch)
//    return 0;
//}

void RCOutput_Ocius::home_mast() {
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
            // Home mast on stinger
        if (mast_status.homed == AP_HAL::SERVO_HOMING) {
            // Wait for current homing to complete.
            return;
        }
//        printf("RCOutput_Ocius::home_mast() - Homing mast on Stinger.\n");
//        gcs().send_text(MAV_SEVERITY_NOTICE, "Homing mast on Stinger.\n");
        uint16_t home_pos = 1900;
        if (pwm_last[BLUEBOTTLE_MAST_CHANN] == 1900) {
            home_pos = 1899;
        }
        mast_status.homed = AP_HAL::SERVO_HOMING;
        SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_CHANN, home_pos);
    }
}


AP_HAL::ServoStatus RCOutput_Ocius::read_status(uint8_t chan) {
    // TODO: RCOutput_Ocius::read_status(uint8_t chan)
    return pwm_status[chan];
}

void RCOutput_Ocius::read_status(AP_HAL::ServoStatus* status, uint8_t len) {
    // TODO: RCOutput_Ocius::read_status(AP_HAL::ServoStatus* status, uint8_t len)
}

void RCOutput_Ocius::home_sail() {
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        printf("RCOutput_Ocius::home_sail() - Homing sail on Stinger.\n");
        // Home sail on stinger
        thread_flags = thread_flags | HomeSail;
    }
}

#define GPIO_PIN 18

void RCOutput_Ocius::motor_status_check(void) {
    if (rover.g2.frame_class == FRAME_BLUEBOTTLE) {
        // disable relay signal for mast
//		printf("Checking if mast signal to be disabled. (%u, %u)\n", timeMastSignalStarted, millis());
        if (timeMastSignalStarted != 0 &&
                (millis() - timeMastSignalStarted) > BLUEBOTTLE_MAST_RELAY_DURATION) {
            // Signal has passed desired time / kill it
            printf("RCO_Ocius: Time up. Mast homed.\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "Mast homed (%s)",
		pwm_last[BLUEBOTTLE_MAST_CHANN] > 1800 ? "up":"down");
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_LOWER_CHANN, 1100);
            SRV_Channels::set_output_pwm_chan(BLUEBOTTLE_MAST_RAISE_CHANN, 1100);
            mast_status.homed = AP_HAL::SERVO_HOMED;
            mast_status.moving = false;
            mast_status.pwm = pwm_last[BLUEBOTTLE_MAST_CHANN];
            timeMastSignalStarted = 0;
        }
    }
}

//void RCOutput_Ocius::motor_status_read(void){
////  DEBUGV("Reading motor status.\n");
//}

#define BLUEBOTTLE_THREAD_PERIOD_WAIT      200000
#define BLUEBOTTLE_MOTOR_OFFSET            (rover.g2.sailboat.sail_epos_zero) // TODO: Switch to using a parameter
#define BLUEBOTTLE_MOTOR_TICKS_PER_90      41000
#define WINCH_ENCODER_DEPLOY		(rover.g2.winch.encoder_out)
#define WINCH_ENCODER_RETRACT       (rover.g2.winch.encoder_in)
#define WINCH_ENCODER_RANGE   (WINCH_ENCODER_DEPLOY - WINCH_ENCODER_RETRACT)

bool bridge_initialised = false;
uint32_t last_initialise_fail_message = 0;
void RCOutput_Ocius::stinger_sail_comm_thread() {
    while (!closing) {
        if (rover.g2.frame_class != FRAME_BLUEBOTTLE) {
            // Not talking to Stinger so sleep and try again.
            goto THREAD_LOOP_SLEEP;
        }

	motor_status_check();

        // initialise bridge
        if (!bridge_initialised) {
            std::string bridgeAddr = "10.42." + std::to_string(rover.g.sysid_this_mav) + ".104:20001";
            if (initializeBridge(0x04, bridgeAddr.c_str())) {
                // Initialisation failed.  Wait 1s and try again.
                uint32_t now = millis();
                if (last_initialise_fail_message == 0 || (now - last_initialise_fail_message) > 10000) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: Unable to initialise bridge connection.\n");
                    last_initialise_fail_message = now;
                }
                usleep(1000000);
                goto THREAD_LOOP_SLEEP;
            }
            bridge_initialised = true;
            gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: Epos bridge initialised.\n");
            last_initialise_fail_message = 0;
            sail_status.homed = AP_HAL::SERVO_UNHOMED;
            winch_status.homed = AP_HAL::SERVO_UNHOMED;
        }


        if (motor_enabled[BLUEBOTTLE_SAIL_CHANN] && mast_status.homed == AP_HAL::SERVO_HOMED && mast_status.pwm <= 1200) {
            // mast is homed & down, sail enabled, so disable
            printf("Stinger: disabling sail motor.\n");
            if (!disableMotor(1)) {
                // Motor successfully disabled.
                motor_enabled[BLUEBOTTLE_SAIL_CHANN] = false;
            }
//            goto THREAD_LOOP_SLEEP;
        }

        stinger_sail_update_epos(sail_status, BLUEBOTTLE_SAIL_CHANN, 1);
        if (rover.g2.winch.enable && WINCH_ENCODER_DEPLOY != WINCH_ENCODER_RETRACT) {
            stinger_sail_update_epos(winch_status, BLUEBOTTLE_WINCH_CHANN, 2);
        }
THREAD_LOOP_SLEEP:
        usleep(BLUEBOTTLE_THREAD_PERIOD_WAIT);
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

#define MOTOR_NAME (nodeid == 2?"Winch":"Sail")

void RCOutput_Ocius::stinger_sail_update_epos(AP_HAL::ServoStatus& motor, uint8_t ch, uint8_t nodeid) {
    if (!bridge_initialised) {
        return;
    }

    uint8_t homed;
    int32_t position;
    uint16_t position_pwm;


    // Get motor position
    if (readPosition(nodeid, &position)) {
        // Error reading motor position.
        consecutive_failures[ch]++;
        if (consecutive_failures[ch] >= MAX_CONSEQ_FAILS) {
            consecutive_failures[ch] = MAX_CONSEQ_FAILS;
            if (motor.homed != AP_HAL::SERVO_UNHOMED) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "RCOut: Consecutive fails on servo %u, unhoming motor", (uint32_t)ch);
            }
            motor.homed = AP_HAL::SERVO_UNHOMED;

            if (RCOutput_Ocius_stinger_epos_all_broken(consecutive_failures, (int)_channel_count)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: Too many consecutive read failures. Resetting bridge connection.");
                printf("RCOut: Too many consecutive read failures. Resetting bridge connection.\n");
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

    if (consecutive_failures[ch] > 0) {
        printf("RCOut: Successful read, clearing consecutive failures.\n");
    }
    consecutive_failures[ch] = 0;

    position_pwm = 0;
    if (ch == BLUEBOTTLE_SAIL_CHANN) {
        position_pwm = ((position - BLUEBOTTLE_MOTOR_OFFSET) * 400 / BLUEBOTTLE_MOTOR_TICKS_PER_90) + 1500;  // Sail motor
    } else if (ch == BLUEBOTTLE_WINCH_CHANN) {
        position_pwm = 1900 - (((position - WINCH_ENCODER_RETRACT)/ (float)WINCH_ENCODER_RANGE) * 800);
    }
    motor.moving = abs(motor.raw - position) > 5;  // if the raw reading has changes by 5 or more, the motor is considered as moving
    motor.raw = position;
    motor.pwm = position_pwm;
	//printf("EPOS %d is at position %d(%d)\n", nodeid, position, position_pwm);

    if (!(motor.moving)) {
        // Check motor is homed
        homed = false;
//        if (motor.homed != AP_HAL::SERVO_HOMED)
//            gcs().send_text(MAV_SEVERITY_WARNING, "Stinger: Checking home status of motor %d( %d )", ch, (int)motor.homed);
        if (isHomed(nodeid, &homed, NULL)) {
            // Error checking motor homed status
//            goto THREAD_LOOP_SLEEP;
            return;
        }

        //printf("Epos(%d) status is %s\n", nodeid, (homed?"homed":"UNhomed"));
        if (homed) {
            // Sail is homed
            if (motor.homed != AP_HAL::SERVO_HOMED) {
                printf("%s is homed.\n", MOTOR_NAME);
                gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: %s is homed.", MOTOR_NAME);
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
 //            printf("Stinger: motor_enabled: %d, mast_flag: %d, mast_pos: %d\n",
//      motor_enabled, mast_status.flag, mast_status.position);

    if (motor.homed != AP_HAL::SERVO_HOMED || (ch == BLUEBOTTLE_SAIL_CHANN && (thread_flags & HomeSail))) {
        if (motor.homed != AP_HAL::SERVO_HOMING) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RCOut: homing %s.", MOTOR_NAME);
            printf("RCOut: homing %s.\n", MOTOR_NAME);
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

//    if (mast_status.position < 1800) {
//        // The mast is not up. FIXME
//        goto THREAD_LOOP_SLEEP;
//        return;
//    }

    if (ch == BLUEBOTTLE_WINCH_CHANN && pwm_last[ch] == 0) {
        // have emergency stop
        if (motor_enabled[ch]) {
            printf("RCOut: Emergency stop called on winch.\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "RCOut: Received emergency stop for winch.\n");
        }
        // disable motor power
        disableMotor(nodeid);
        // Set target position to current position
        moveToPosition(nodeid, position);
        motor_enabled[ch] = false;
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
                desiredPosition = (pwm_last[ch] - 1500) * BLUEBOTTLE_MOTOR_TICKS_PER_90 / 400 + BLUEBOTTLE_MOTOR_OFFSET;  // FIXME
            }
            if (!rover.g2.sailboat.sail_is_safe() && desiredPosition != BLUEBOTTLE_MOTOR_OFFSET) {
                // if the sail is not safe it can only be centered, in preparation for stowing
                desiredPosition = position;
            }
        } else if (ch == BLUEBOTTLE_WINCH_CHANN) {
            desiredPosition = ((1900 - pwm_last[ch]) / 800.0) * WINCH_ENCODER_RANGE + WINCH_ENCODER_RETRACT;
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
                printf("RCOut: Moving %s to position %d.\n", MOTOR_NAME, desiredPosition);
                last_move_attempt[ch] = desiredPosition;
                if (!moveToPosition(nodeid, desiredPosition)) {
                    last_move_success[ch] = desiredPosition;
                    last_move_time[ch] = now;
                }
            }
        }
    }
}
