/*
  Copyright 2016-2017 Nathan Kau nathankau@stanford.edu

  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "VESC-tutorial-project.h"
#include <FlexCAN.h>
#include "AngularPDController.h"
#include "buffer.h"
#include "utils.h"
#include "VESC.h"
#include "comm_can.h"

/******** GLOBAL VARISBLES *********/

// Create CAN object:
// 1st param: 500kbps
// 2nd param: Use CAN0
// 3rd and 4th params: Use alt tx and alt rx pins
FlexCAN CANTransceiver(500000,0,1,1);

// Variable to keep track of the last time a debugging print message was sent
// The elapsedMicros type automatically increments itself every loop execution!
elapsedMillis last_print_shit;

// Variable to keep track of time since command to RM was sent
// The elapsedMicros type automatically increments itself every loop execution!
elapsedMicros vesc1_time_since_command = 0;

// VESC motor objects
VESC vesc1(CANTransceiver); // CAN flexcan

// STATE MACHINE STATE VARIABLE
enum controller_state_machine {
	STAGING,
	RUNNING,
	ESTOP
};
controller_state_machine controller_state = STAGING;

// Keep track of when RUNNING was entered
long running_timestamp;

/********* CONSTANTS *********/

// Send position commands at 500hz (1s / 2000us)
const int UPDATE_PERIOD =  2000; // us
// const int UPDATE_PERIOD =  10000; // us


// built-in led pin
int led_pin = 13;
#define LED_ON digitalWrite(led_pin,HIGH)
#define LED_OFF digitalWrite(led_pin,LOW)

const float MAX_CURRENT = 8.0; // 30 amps seems the max

const int8_t VESC1_CHANNEL_ID = 0;

const float VESC1_OFFSET = -108; // 108
const int VESC1_DIRECTION = -1;

/****************************/


/**
 * Call this function in the main loop if you want to see the normalized motor angles
 */
void print_shit() {
  if(last_print_shit > 10) {
    last_print_shit -= 10;

    // Serial.println(loop_time);
    Serial.println(vesc1.read());
  }
}

void transition_to_running() {
  Serial.println("Transitioning to RUNNING");
  running_timestamp = millis();
  controller_state = RUNNING;
}
void transition_to_ESTOP() {
  Serial.println("Transitioning to ESTOP");
  controller_state = ESTOP;
}
void transition_to_STAGING() {
  Serial.println("Transitioning to STAGING");
  controller_state = STAGING;

  // Program specific resets
  reset_impulse();
}

/**
 * Process serial commands send from a computer
 */
void process_serial() {
  if(Serial.available()) {
    char c = Serial.read();

    switch(c) {
      // Send 's' over serial to permanently ESTOP
      case 's':
        transition_to_ESTOP();
        break;
      // Send 'b' over serial to begin running if not in ESTOP mode
      case 'b':
        if(controller_state != ESTOP) {
          transition_to_running();
        }
        break;
      // Send 'r' over serial to reset this program's vars and restart
      case 'r':
        transition_to_STAGING();
        break;

      default:
        break;
    }

    // Clear the buffer after the first byte is read.
    // So don't send multiple commands at once and expect any but the first to be executed
    Serial.clear();
  }
}

/**
 * Handle any position messages from VESCs
 */
void process_CAN_messages() {
  float last_read_angle;
  int transmitter_ID;

  // time to read angle over can is 9 us
  if(readAngleOverCAN(CANTransceiver, last_read_angle, transmitter_ID)) {
    switch(transmitter_ID) {
      case VESC1_CHANNEL_ID:
        vesc1.update_angle(last_read_angle);
        break;
    }
  }
}

void setup() {
  // Initialize CAN bus
  CANTransceiver.begin();
  Serial.println("CAN Transmitter Initialized");

  // Initialize "power on" led
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);

  // Wait for shit to get set up
  delay(1000);

  // Initialize VESC controller objects
  vesc1.attach(VESC1_CHANNEL_ID,
                  VESC1_OFFSET,
                  VESC1_DIRECTION,
                  MAX_CURRENT);

  Serial.begin(115200);

}

void STAGING_STATE() {
  // vesc1.write_current(0.0f);
}

// Call this function in RUNNING_STATE to move the motor in a sinusoid
void sinusoid() {
  float seconds = (float)(millis()%1000000) / 1000.0;
  float freq = 0.25; // actual freq, not angular freq
  // Must use sinf and not sin??
  float motor_angle = sinf(seconds*2*PI*freq) * 180.0;

  // This particular set of arguments sets Kd to 0.05, Ki to 0, Kd to 0.0005, and target position to 0 degrees
  vesc1.write_pos_and_pid_gains(0.05, 0, 0.0005, motor_angle);
}

static bool impulse_started = false;
static bool impulse_finished = false;
static bool impulsed_printed_finish = false;

void impulse_print_position() {
  Serial.println(vesc1.read());
}

void reset_impulse() {
  impulse_finished = false;
  impulsed_printed_finish = false;
  impulse_started = false;
}

void impulse() {
  // Print IMPULSE-STARTING when the function is first called
  if(!impulse_started) {
    Serial.println("IMPULSE-STARTING");
    impulse_started = true;
  }

  // Print IMPULSE-DONE after the program is finished
  if(impulse_finished && !impulsed_printed_finish) {
    Serial.println("IMPULSE-DONE");
    impulsed_printed_finish = true;
  }

  // Send motor position command based off which time segment we're in
  static long time_running = 0;
  time_running = millis() - running_timestamp;

  // Send to 180 deg and wait for 500ms for the motor to get there)
  if (time_running < 500) {
    vesc1.write(180.0f);
    return;
  }

  // Send zero current after the impulse is done
  if(time_running > 2000) {
    impulse_finished = true;
    LED_ON;
    transition_to_ESTOP();
    return;
  }

  // If execution reaches, here it means we're in the actual impulse stage
  // Print the motor position

  // CAITLIN: using this function makes it not work
  // impulse_print_position();
  // CAITLIN: Printing directly works
  // Does not work when Serial.print
  Serial.println(vesc1.read());

  // Set 180.0 deg position and Wait 1 second to measure resting instability
  if (time_running < 1000) {
    LED_OFF;
    vesc1.write(180.0f);
    return;
  }

  // Send to 90 deg and wait for 500ms
  if(time_running >= 1000 && time_running < 1500) {
    LED_ON;
    // Serial.println("");
    vesc1.write(90.0f);
    return;
  }

  /***** CAITLIN THE BELOW ISNT WORKING ******/
  // THINGS TRIED
  // Filling out 8 byte can message buffer instead of filling out first 4 bytes
  // Printing out CAN message data in binary and making sure its not corrupted
  // Can't make the CAN_messate_t variable global or static :(

  // Send back to 0 deg and wait for 500 ms
  if(time_running >= 1500 && time_running < 2000) {
    // Serial.println("");

    // Without the above println, the LED_OFF which uses digitalWrite DOES WORK
    LED_OFF;

    // Without the above println, the vesc1.write DOES NOT WORK
    vesc1.write(180.0f);
    return;
  }
}

void RUNNING_STATE() {
  /****** Send current messages to VESCs *******/
  // Send position current commands at 200khz aka 5000 us per loop

  // Send VESC 1 position command
  if(vesc1_time_since_command > UPDATE_PERIOD) {
    vesc1_time_since_command = 0;

    impulse();
  }
  /****** End of sending current messages to VESCs *******/
}

void ESTOP_STATE() {
  vesc1.write_current(0.0f);
  delay(100);
}

void loop() {
    // Process any CAN messages from the motor controllers
    process_CAN_messages();

    // IMPORTANT: read any commands sent by the computer
    process_serial();

    // IMPORTANT: for some reason the code doesn't work without this function call
    // Probably has to do with a delay thing
    // print_shit();

    switch(controller_state) {
      case STAGING:
        STAGING_STATE();
        break;
      case RUNNING:
        RUNNING_STATE();
        break;
      case ESTOP:
        ESTOP_STATE();
        break;
      default:
        ESTOP_STATE();
        break;
    }
}
