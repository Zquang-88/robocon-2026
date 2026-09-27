// Standalone commissioning firmware for the route immediately after the bridge.
//
// Upload with:
//   platformio run -e after_bridge_test -t upload
//
// Place the robot after the descending ramp with the center eight-eye array on
// the longitudinal guide line. Press the physical START button on Teensy pin 34.
// The robot runs the complete post-bridge route through THA_2A and the THA_2B
// line approach. It stops and disarms immediately after both side three-eye
// clusters have left the transverse black line.

#define ROBOCON_AFTER_BRIDGE_TEST_BUILD 1
#define ROBOCON_AFTER_BRIDGE_TEST_STOP_AFTER_SIDE_CLEAR 1
#define ROBOCON_AFTER_BRIDGE_TEST_CHASSIS_ONLY 1
#include "main.cpp"
