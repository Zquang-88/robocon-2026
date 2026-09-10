// Standalone commissioning firmware for the route immediately after the bridge.
//
// Upload with:
//   platformio run -e after_bridge_test -t upload
//
// Place the robot after the descending ramp with the center eight-eye array on
// the longitudinal guide line. Press the physical START button on Teensy pin 34.
// The robot captures the current BNO085 heading, follows the guide, detects the
// transverse black marker, then advances 400 mm for a center-array detection or
// 300 mm for a two-side-cluster detection before stopping and disarming.

#define ROBOCON_AFTER_BRIDGE_TEST_BUILD 1
#include "main.cpp"
