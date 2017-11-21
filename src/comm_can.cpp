#include <FlexCAN.h>
#include "buffer.h"

static CAN_message_t msg;

/**
 * Check for and read motor angles over CAN.
 * Returns true and stores value if message received, otherwise return false
 * TODO: Test how clearing or not clearing the message buffer affects stability
 **/
bool readAngleOverCAN(FlexCAN& CANrx, float& last_angle_received, int& transmitter_ID) {
  // keep track if a value was received
  bool read = false;

  // parse all available can messages
  while( CANrx.read(msg)) {
    read = true;

    // Very useful, this is the ID encoded by the transitting CAN device
    transmitter_ID = msg.id & 0xFF;

    // dummy variable for buffer get functino
    int32_t index=0;

    // parse a 32bit float from the can message data buffer
    last_angle_received = buffer_get_float32(msg.buf, 100000, &index);
    return read;
  }
  return read;
}
