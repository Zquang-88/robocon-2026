#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include "UartProtocol.h"
#include "Wire.h"
#include "HardwareConfig.h"

uint64_t mockMicros = 0;
bool PersistentConfig::save(MechanismConfig &, char *, size_t) { return false; }

struct TestStream : Stream {
  std::deque<char> input;
  std::string output;
  int available() override { return static_cast<int>(input.size()); }
  int read() override { const char c = input.front(); input.pop_front(); return c; }
  size_t println(const char *text) override { output += text; output += '\n'; return strlen(text) + 1; }
  void feed(const std::string &text) { for (char c : text) input.push_back(c); }
};

struct Rig {
  AccelStepper a{AccelStepper::DRIVER, 1, 2}, b{AccelStepper::DRIVER, 45, 48};
  ValveController valves;
  MechanismConfig config;
  PersistentConfig storage;
  TestStream usb, robot;
  MechanismController mechanism{a, b, valves};
  UartProtocol protocol{usb, robot, mechanism, config, storage};
  Rig(bool valveHeldAtBoot = false, bool homeHeldAtBoot = false) {
    resetMockDigitalPins();
    setMockDigitalPin(HardwareConfig::VALVE_BUTTON_PIN, valveHeldAtBoot ? LOW : HIGH);
    setMockDigitalPin(HardwareConfig::HOME_BUTTON_PIN, homeHeldAtBoot ? LOW : HIGH);
    Wire.reset(); loadDefaultMechanismConfig(config); mechanism.begin(config);
    protocol.begin(); advance(30);
  }
  void command(const std::string &line) { robot.feed(line + "\n"); protocol.update(); }
  void advance(uint32_t ms) {
    const auto until = mockMicros + uint64_t(ms) * 1000;
    while (mockMicros < until) { mockMicros += 100; protocol.update(); }
  }
  void tap(uint8_t pin) {
    setMockDigitalPin(pin, LOW); advance(40);
    setMockDigitalPin(pin, HIGH); advance(40);
  }
};

int main() {
  {
    Rig r; r.command("JOG,-500,0");
    for (int i = 0; i < 5; ++i) {
      r.advance(120);
      const float before = r.a.speed();
      r.command("JOG,-500,0");
      assert(std::fabs(r.a.speed() - before) < 1.0f);
    }
    assert(r.a.currentPosition() < -150 && r.b.currentPosition() == 0);
    r.command("JOG,-500,700"); r.advance(150);
    assert(r.b.currentPosition() > 0);
    r.command("JOG,0,700"); const long stoppedA = r.a.currentPosition();
    const long movingB = r.b.currentPosition(); r.advance(120);
    assert(r.a.currentPosition() == stoppedA && r.b.currentPosition() > movingB);
    r.command("JOG,STOP"); const long stoppedB = r.b.currentPosition(); r.advance(700);
    assert(r.b.currentPosition() == stoppedB && !r.mechanism.busy());
    char event[32]; assert(!r.mechanism.takeDoneEvent(event, sizeof(event)));
    puts("PASS: acceleration refresh, independent axes, immediate release, no AUTO DONE");
  }
  {
    Rig r; r.command("JOG,500,0"); r.advance(510);
    assert(r.mechanism.fault() == MechanismFault::JogLinkTimeout);
    const long atStop = r.a.currentPosition(); r.command("JOG,500,0"); r.advance(50);
    assert(r.a.currentPosition() == atStop && !r.mechanism.busy());
    r.command("CLEAR_FAULT"); r.command("JOG,500,0"); r.advance(100);
    assert(r.a.currentPosition() > atStop); r.command("STOP");
    puts("PASS: watchdog stops and late packets cannot restart without reset");
  }
  {
    Rig r; r.command("JOG,500,0"); r.advance(100);
    mockMicros += 600000; r.command("JOG,500,0");
    assert(r.mechanism.fault() == MechanismFault::JogLinkTimeout);
    puts("PASS: timeout is checked before a delayed UART refresh");
  }
  {
    Rig r; r.config.maximumPositionAmm = 3; r.config.minimumPositionAmm = -3;
    r.command("JOG,500,0");
    for (int i = 0; i < 8; ++i) { r.advance(120); r.command("JOG,500,0"); }
    assert(r.mechanism.positionAmm() <= 3.0f && r.mechanism.positionAmm() >= 2.8f);
    r.command("JOG,-500,0");
    for (int i = 0; i < 8; ++i) { r.advance(120); r.command("JOG,-500,0"); }
    assert(r.mechanism.positionAmm() >= -3.0f && r.mechanism.positionAmm() <= -2.8f);
    r.command("JOG,STOP"); puts("PASS: both software limits and reversal");
  }
  {
    Rig r;
    for (const char *bad : {"JOG,NAN,0", "JOG,INF,0", "JOG,30001,0", "JOG,1,0", "JOG,500,0junk", "JOG,500"}) {
      r.command(bad); assert(!r.mechanism.busy());
    }
    r.command("CMD,PICK_A"); assert(r.mechanism.busy());
    r.command("JOG,500,0"); assert(!r.mechanism.jogging());
    r.command("STOP"); r.advance(200); assert(!r.mechanism.busy());
    // A suffix inside an oversized record must never become a motion command.
    r.robot.feed(std::string(256, 'X') + "JOG,500,0\n"); r.advance(2);
    assert(!r.mechanism.busy());
    puts("PASS: invalid input, sequence interlock, STOP and oversized record");
  }
  {
    Rig r;
    r.command(std::string("\xFF", 1) + "ROBOT START");
    assert(r.mechanism.busy());
    assert(strcmp(r.mechanism.activeProfileName(), "ROBOT_START") == 0);
    r.command("STOP");
    puts("PASS: leading non-ASCII UART noise is stripped before ROBOT START");
  }
  {
    Rig r;
    const long startA = r.a.currentPosition(), startB = r.b.currentPosition();
    r.command("SIM,POINT_A"); r.advance(1000);
    assert(!r.mechanism.busy());
    assert(r.a.currentPosition() == startA && r.b.currentPosition() == startB);
    assert(r.robot.output.find("ERR,UNKNOWN_COMMAND,SIM,POINT_A") != std::string::npos);
    puts("PASS: removed SIM commands are rejected without motor motion");
  }
  {
    Rig r;
    r.command("POINT_B");
    for (int i = 0; i < 20000 && r.mechanism.activeStep() < 2; ++i)
      r.advance(1);
    assert(r.mechanism.activeStep() == 2);
    assert(std::fabs(r.mechanism.positionAmm() -
                     (HardwareConfig::READY_HOME_A_POSITION_A_MM +
                      HardwareConfig::PICK_LOWER_DIRECTION_A *
                          HardwareConfig::RED_POINT_B_LOWER_DISTANCE_A_MM)) < 0.1f);
    assert(std::fabs(r.mechanism.positionBmm() -
                     (HardwareConfig::READY_HOME_A_POSITION_B_MM +
                      HardwareConfig::PICK_LOWER_DIRECTION_B *
                          HardwareConfig::RED_POINT_B_LOWER_DISTANCE_B_MM)) < 0.1f);
    for (int i = 0; i < 40000 &&
                    r.robot.output.find("DONE_POINT_B") == std::string::npos;
         ++i) r.advance(1);
    assert(r.robot.output.find("DONE_POINT_B") != std::string::npos);
    assert(r.mechanism.busy());
    assert(strcmp(r.mechanism.activeProfileName(), "PICK_B") == 0);
    assert(r.mechanism.activeStep() == 6);
    assert(std::fabs(r.mechanism.positionAmm() -
                     HardwareConfig::READY_HOME_A_POSITION_A_MM) < 5.0f);
    assert(std::fabs(r.mechanism.positionBmm() -
                     HardwareConfig::READY_HOME_A_POSITION_B_MM) < 5.0f);
    const long aWhenDone = r.a.currentPosition();
    const long bWhenDone = r.b.currentPosition();
    r.advance(150);
    assert(r.a.currentPosition() > aWhenDone);
    assert(r.b.currentPosition() < bWhenDone);
    for (int i = 0; i < 20000 && r.mechanism.activeStep() < 8; ++i)
      r.advance(1);
    assert(r.mechanism.activeStep() == 8);
    const long aDuringSecondHalf = r.a.currentPosition();
    const long bHoldingAtOneThird = r.b.currentPosition();
    r.advance(150);
    assert(r.a.currentPosition() > aDuringSecondHalf);
    assert(r.b.currentPosition() == bHoldingAtOneThird);
    r.advance(20000);
    assert(!r.mechanism.busy());
    assert(std::fabs(r.mechanism.positionAmm() -
                     (HardwareConfig::READY_HOME_A_POSITION_A_MM +
                      HardwareConfig::POINT_B_FINAL_RAISE_DIRECTION_A *
                          HardwareConfig::POINT_B_FINAL_RAISE_DISTANCE_MM)) < 0.1f);
    assert(std::fabs(r.mechanism.positionBmm() -
                     (HardwareConfig::READY_HOME_A_POSITION_B_MM +
                      HardwareConfig::POINT_B_FINAL_RAISE_DIRECTION_B *
                          HardwareConfig::POINT_B_FIRST_STAGE_B_DISTANCE_MM)) < 0.1f);
    assert(r.robot.output.find("DONE,PICK_B") == std::string::npos);
    r.command("BRIDGE_STABLE");
    assert(r.robot.output.find("ACK,STARTED,BRIDGE_B") != std::string::npos);
    for (int i = 0; i < 20000 &&
                    r.robot.output.find("DONE_BRIDGE_B") == std::string::npos;
         ++i) r.advance(1);
    assert(r.robot.output.find("DONE_BRIDGE_B") != std::string::npos);
    assert(!r.mechanism.busy());
    assert(std::fabs(r.mechanism.positionAmm() -
                     (HardwareConfig::READY_HOME_A_POSITION_A_MM +
                      HardwareConfig::POINT_B_FINAL_RAISE_DIRECTION_A *
                          HardwareConfig::POINT_B_FINAL_RAISE_DISTANCE_MM)) < 0.1f);
    assert(std::fabs(r.mechanism.positionBmm() -
                     (HardwareConfig::READY_HOME_A_POSITION_B_MM +
                      HardwareConfig::POINT_B_FINAL_RAISE_DIRECTION_B *
                          HardwareConfig::POINT_B_FINAL_RAISE_DISTANCE_MM)) < 0.1f);
    r.command("BRIDGE_STABLE");
    assert(r.robot.output.rfind("DONE_BRIDGE_B") != std::string::npos);
    puts("PASS: POINT_B raises A=1700/B=566.7, then BRIDGE_STABLE finishes B=1700");
  }
  {
    Rig r;
    assert(Wire.sdaPin == 8 && Wire.sclPin == 3);
    assert(Wire.currentAddress == 0x20 && Wire.lastWritten == 0xFF);
    r.command("PCF,P0,ON");
    assert(Wire.lastWritten == 0xFF);  // break: P0/P7 both HIGH
    r.advance(49);
    assert(Wire.lastWritten == 0xFF);
    r.advance(1);
    assert(Wire.lastWritten == 0xFE && r.mechanism.valveMask() == 0x01);
    r.command("PCF,P7,ON");
    assert(Wire.lastWritten == 0xFF);  // break before reversing the pair
    r.advance(50);
    assert(Wire.lastWritten == 0x7F && r.mechanism.valveMask() == 0x80);
    r.command("PCF,P0,OFF");
    assert(Wire.lastWritten == 0x7F);  // OFF P0 selects opposite P7
    r.command("PCF,ALL,OFF");
    assert(Wire.lastWritten == 0xFF && r.mechanism.valveMask() == 0x00);
    for (uint8_t state : Wire.writes) {
      assert((state & 0x81) != 0 && (state & 0x42) != 0);
      assert((state & 0x24) != 0 && (state & 0x18) != 0);
    }
    r.command("PCF,P8,ON");
    assert(r.robot.output.find("ERR,PCF_FORMAT_USE_P0_TO_P7_ON_OFF") !=
           std::string::npos);
    puts("PASS: PCF address/pins, active-low pairs, 50 ms deadtime and all-off");
  }
  {
    Rig r;
    r.command("POINT_A");
    r.command("POINT_B");
    assert(r.robot.output.find("BUSY") != std::string::npos);
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_A") == std::string::npos;
         ++i) r.advance(1);
    assert(r.robot.output.find("[POINT_A] P2 + P3 ON TOGETHER") != std::string::npos);
    bool sawPointABothOn = false;
    for (uint8_t state : Wire.writes) {
      const bool p2On = (state & (1U << 2)) == 0;
      const bool p3On = (state & (1U << 3)) == 0;
      assert(p2On == p3On);
      if (p2On && p3On) sawPointABothOn = true;
    }
    assert(sawPointABothOn);
    assert(r.robot.output.find("DONE_POINT_A") != std::string::npos);
    assert(Wire.lastWritten == 0xF3);  // RED POINT_A: P2/P3 LOW
    r.command("STOP");
    assert(Wire.lastWritten == 0xFF);
    assert(r.robot.output.find("STOPPED") != std::string::npos);
    puts("PASS: non-blocking POINT_A ordering, BUSY and immediate STOP");
  }
  {
    Rig r;
    r.command("THA_2A");
    for (int i = 0; i < 5000 &&
                    r.robot.output.find("DONE_THA_2A") == std::string::npos;
         ++i) r.advance(1);
    assert(Wire.lastWritten == 0xAF);  // P6/P4 LOW; all others HIGH
    assert(r.robot.output.find("[THA_2A] P1 OFF, P6 ON") != std::string::npos);
    assert(r.robot.output.find("[THA_2A] P3 OFF, P4 ON") != std::string::npos);
    puts("PASS: THA_2A exact pair order and response");
  }
  {
    Rig r;
    r.command("THA_2B");
    r.advance(50);
    assert(Wire.lastWritten == 0xDF);  // P2 OFF selects opposite P5 ON
    r.advance(349);
    assert(Wire.lastWritten == 0xDF);  // P7 must still be OFF
    r.advance(1);
    assert(Wire.lastWritten == 0xDF);  // P0/P7 transition starts after 350 ms
    r.advance(50);
    assert(Wire.lastWritten == 0x5F);  // P5 and P7 ON
    for (int i = 0; i < 5000 &&
                    r.robot.output.find("DONE_THA_2B") == std::string::npos;
         ++i) r.advance(1);
    assert(r.robot.output.find("[THA_2B] P2 OFF, P5 ON") != std::string::npos);
    assert(r.robot.output.find("[THA_2B] Wait 350 ms") != std::string::npos);
    assert(r.robot.output.find("[THA_2B] P0 OFF, P7 ON") != std::string::npos);
    puts("PASS: RED THA_2B P2 OFF then P0 OFF with a verified 350 ms interval");
  }
  {
    Rig r;
    r.command("FIELD,BLUE");
    r.robot.output.clear();
    r.command("THA_2B");
    r.advance(50);
    assert(Wire.lastWritten == 0x7F);  // BLUE: P0 OFF selects opposite P7 ON
    r.advance(349);
    assert(Wire.lastWritten == 0x7F);  // P5 must still be OFF
    r.advance(1);
    assert(Wire.lastWritten == 0x7F);  // P2/P5 transition starts after 350 ms
    r.advance(50);
    assert(Wire.lastWritten == 0x5F);  // P7 and P5 ON
    assert(r.robot.output.find("[THA_2B] P0 OFF, P7 ON") != std::string::npos);
    assert(r.robot.output.find("[THA_2B] Wait 350 ms") != std::string::npos);
    assert(r.robot.output.find("[THA_2B] P2 OFF, P5 ON") != std::string::npos);
    puts("PASS: BLUE THA_2B P0 OFF then P2 OFF with a verified 350 ms interval");
  }
  {
    Rig r;
    r.command("POINT_B");
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_B") == std::string::npos;
         ++i) r.advance(1);
    assert(Wire.lastWritten == 0xFC);  // RED POINT_B: P0/P1 LOW
    assert(r.robot.output.find("[POINT_B] P0 + P1 ON TOGETHER") != std::string::npos);
    bool sawPointBBothOn = false;
    for (uint8_t state : Wire.writes) {
      const bool p0On = (state & (1U << 0)) == 0;
      const bool p1On = (state & (1U << 1)) == 0;
      assert(p0On == p1On);
      if (p0On && p1On) sawPointBBothOn = true;
    }
    assert(sawPointBBothOn);
    puts("PASS: RED POINT_B switches P0/P1 together and responds");
  }
  {
    Rig r;
    r.command("POINT_C");
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_C") == std::string::npos;
         ++i) r.advance(1);
    const size_t firstDone = r.robot.output.find("DONE_POINT_C");
    assert(firstDone != std::string::npos);
    assert(strcmp(r.mechanism.activeProfileName(), "PICK_C") == 0);
    assert(Wire.lastWritten == 0xFC);  // RED POINT_C reuses POINT_B valve pair.
    assert(r.robot.output.find("[POINT_C] P0 + P1 ON TOGETHER") != std::string::npos);
    r.advance(500);
    assert(r.robot.output.find("DONE_POINT_C", firstDone + 1) == std::string::npos);
    while (r.mechanism.busy()) r.advance(1);
    r.command("BRIDGE_STABLE");
    assert(r.robot.output.find("ACK,STARTED,BRIDGE_B") != std::string::npos);
    puts("PASS: POINT_C mirrors POINT_B, responds once and enables BRIDGE_STABLE");
  }
  {
    Rig r;
    assert(r.robot.output.find("ACK,FIELD,RED") != std::string::npos);
    assert(mockDigitalPinState[HardwareConfig::RGB_RED_PIN] == HIGH);
    assert(mockDigitalPinState[HardwareConfig::RGB_GREEN_PIN] == LOW);
    assert(mockDigitalPinState[HardwareConfig::RGB_BLUE_PIN] == LOW);
    r.command("FIELD,BLUE");
    assert(r.robot.output.find("ACK,FIELD,BLUE") != std::string::npos);
    assert(mockDigitalPinState[HardwareConfig::RGB_RED_PIN] == LOW);
    assert(mockDigitalPinState[HardwareConfig::RGB_GREEN_PIN] == LOW);
    assert(mockDigitalPinState[HardwareConfig::RGB_BLUE_PIN] == HIGH);
    r.command("POINT_A");
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_A") == std::string::npos;
         ++i) r.advance(1);
    assert(Wire.lastWritten == 0xFC);  // BLUE POINT_A: P0/P1 LOW
    assert(r.robot.output.find("[POINT_A] P0 + P1 ON TOGETHER") != std::string::npos);
    puts("PASS: FIELD BLUE ACK, blue LED and mirrored POINT_A valves");
  }
  {
    Rig r;
    r.command("FIELD,BLUE");
    r.command("POINT_B");
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_B") == std::string::npos;
         ++i) r.advance(1);
    assert(Wire.lastWritten == 0xF3);  // BLUE POINT_B: P2/P3 LOW
    assert(r.robot.output.find("[POINT_B] P2 + P3 ON TOGETHER") != std::string::npos);
    puts("PASS: BLUE POINT_B switches mirrored P2/P3 pair");
  }
  {
    Rig r;
    Wire.failTransactions = true;
    r.command("PCF,P2,ON");
    assert(r.mechanism.fault() == MechanismFault::Pcf8574Unavailable);
    assert(r.robot.output.find("ERROR_PCF8574") != std::string::npos);
    puts("PASS: I2C write failure aborts operation and reports ERROR_PCF8574");
  }
  {
    Rig r;
    r.command("POINT_A");
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_A") == std::string::npos;
         ++i) r.advance(1);
    assert(Wire.lastWritten == 0xF3);  // RED POINT_A keeps P2/P3 ON
    r.command("POINT_B");
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_B") == std::string::npos;
         ++i) r.advance(1);
    assert(Wire.lastWritten == 0xF0);  // adds P0/P1; preserves P2/P3
    while (r.mechanism.busy()) r.advance(1);
    r.command("THA_2A");
    while (r.mechanism.busy()) r.advance(1);
    assert(Wire.lastWritten == 0xAA);  // P0/P2/P4/P6 LOW
    r.command("THA_2B");
    while (r.mechanism.busy()) r.advance(1);
    assert(Wire.lastWritten == 0x0F);  // P4/P5/P6/P7 LOW
    for (uint8_t state : Wire.writes) {
      assert((state & 0x81) != 0 && (state & 0x42) != 0);
      assert((state & 0x24) != 0 && (state & 0x18) != 0);
    }
    puts("PASS: complete A/B/THA flow preserves unrelated bits and pair safety");
  }
  {
    Rig r;
    setMockDigitalPin(36, HIGH);
    setMockDigitalPin(37, HIGH);
    r.command("THA_2B");
    for (int i = 0; i < 5000 &&
                    r.robot.output.find("DONE_THA_2B") == std::string::npos;
         ++i) r.advance(1);
    assert(r.robot.output.find("DONE_THA_2B") != std::string::npos);
    assert(r.mechanism.busy());
    const long startA = r.a.currentPosition();
    const long startB = r.b.currentPosition();
    r.advance(150);
    assert(r.a.currentPosition() == startA);  // A held during B pre-lower
    assert(r.b.currentPosition() > startB);   // B lowers first

    for (int i = 0; i < 10000 && r.mechanism.activeStep() == 2; ++i)
      r.advance(1);
    assert(r.mechanism.activeStep() == 3);
    const long bAfterPrelower = r.b.currentPosition();
    assert(r.a.currentPosition() >= startA - 10);  // transition-cycle tolerance
    assert(bAfterPrelower > startB);
    r.advance(150);
    assert(r.a.currentPosition() < startA);         // synchronized lowering
    assert(r.b.currentPosition() > bAfterPrelower); // B continues downward

    setMockDigitalPin(37, LOW);
    r.advance(30);
    assert(r.b.currentPosition() == 0);
    const long aAtBHome = r.a.currentPosition();
    r.advance(150);
    assert(r.a.currentPosition() > aAtBHome);  // A reverses toward HOME36
    assert(r.b.currentPosition() == 0);

    setMockDigitalPin(36, LOW);
    r.advance(30);
    assert(!r.mechanism.busy() && r.mechanism.zeroed());
    assert(r.a.currentPosition() == 0 && r.b.currentPosition() == 0);
    puts("PASS: THA_2B lowers B 400 mm, lowers A/B to HOME37, then homes A to HOME36");
  }
  {
    Rig r;
    // Nothing arrives on either Serial port: both buttons work offline.
    r.tap(11); r.advance(60); r.tap(11); r.advance(60);
    assert(Wire.lastWritten == 0xF0);  // work coils P0-P3 all on
    for (uint8_t state : Wire.writes) {
      assert((state & 0x81) && (state & 0x42));
      assert((state & 0x24) && (state & 0x18));
    }
    r.tap(11);
    r.advance(20);  // allow the 50 ms break-before-make interval to finish
    assert(Wire.lastWritten == 0x0F);  // P4-P7 ON: all valves closed
    assert(r.robot.output.find("ACK,BUTTON11,P4_P7_ON_CLOSED") != std::string::npos);
    r.advance(1000); assert(Wire.lastWritten == 0x0F);
    puts("PASS: GPIO11 double press opens P0-P3; single press closes with P4-P7 offline");
  }
  {
    Rig r(true, true);
    r.advance(1200);
    assert(!r.mechanism.busy() && Wire.lastWritten == 0xFF);
    assert(r.robot.output.find("ACK,BUTTON11,") == std::string::npos);
    assert(r.robot.output.find("ACK,BUTTON12,") == std::string::npos);
    setMockDigitalPin(11, HIGH); setMockDigitalPin(12, HIGH); r.advance(40);
    for (int i = 0; i < 5; ++i) {
      setMockDigitalPin(11, LOW); setMockDigitalPin(12, LOW); r.advance(5);
      setMockDigitalPin(11, HIGH); setMockDigitalPin(12, HIGH); r.advance(5);
    }
    r.advance(600);
    assert(r.robot.output.find("ACK,BUTTON11,") == std::string::npos);
    assert(r.robot.output.find("ACK,BUTTON12,") == std::string::npos);
    puts("PASS: held-at-boot and bouncing buttons cannot start work");
  }
  {
    Rig r;
    setMockDigitalPin(36, HIGH); setMockDigitalPin(37, HIGH);
    r.tap(12); r.advance(150);
    assert(r.mechanism.homing() && !r.mechanism.zeroed());
    assert(r.a.currentPosition() == 0 && r.b.currentPosition() > 0);
    assert(r.a.maxSpeed() == 8000.0f && r.b.maxSpeed() == 8000.0f);
    assert(r.a.acceleration() == 5500.0f && r.b.acceleration() == 5500.0f);
    const long bAtSwitch = r.b.currentPosition();
    setMockDigitalPin(37, LOW); r.advance(10);
    assert(r.b.currentPosition() == bAtSwitch && r.a.currentPosition() == 0);
    // A short HOME37 bounce must not release A.
    setMockDigitalPin(37, HIGH); r.advance(5);
    assert(r.a.currentPosition() == 0);
    setMockDigitalPin(37, LOW); r.advance(30);
    assert(r.b.currentPosition() == 0);
    r.advance(150);
    assert(r.a.currentPosition() > 0 && r.b.currentPosition() == 0);
    setMockDigitalPin(36, LOW); r.advance(30);
    assert(!r.mechanism.busy() && r.mechanism.zeroed());
    assert(r.a.currentPosition() == 0 && r.b.currentPosition() == 0);
    assert(r.robot.output.find("ACK,BUTTON12,HOME_B_THEN_A,8000,5500") != std::string::npos);
    puts("PASS: GPIO12 homes B first, confirms HOME37, then homes A at 8000/5500 offline");
  }
  {
    Rig r;
    // B is already on HOME: never move B, but finish homing A.
    setMockDigitalPin(36, HIGH); setMockDigitalPin(37, LOW);
    r.tap(12); r.advance(150);
    assert(r.b.currentPosition() == 0 && r.a.currentPosition() > 0);
    r.command("STOP");
    const long stoppedA = r.a.currentPosition();
    r.advance(500);
    assert(!r.mechanism.busy() && r.a.currentPosition() == stoppedA);
    assert(!r.mechanism.zeroed());
    puts("PASS: already-HOME B stays still; UART STOP cancels button homing");
  }
  {
    Rig r;
    setMockDigitalPin(36, HIGH); setMockDigitalPin(37, HIGH);
    r.tap(12); r.advance(HardwareConfig::BUTTON_HOME_TIMEOUT_MS);
    assert(r.mechanism.fault() == MechanismFault::HomeTimeout);
    assert(r.a.currentPosition() == 0 && !r.mechanism.busy());
    const long stoppedB = r.b.currentPosition();
    r.tap(12); r.advance(100);
    assert(!r.mechanism.busy() && r.b.currentPosition() == stoppedB);
    puts("PASS: missing HOME37 times out; button cannot bypass a fault");
  }
  {
    Rig r;
    r.command("POINT_A");
    r.tap(12);
    assert(r.mechanism.competitionActionRunning());
    assert(r.robot.output.find("ERR,BUTTON12_HOME_REJECTED,BUSY") != std::string::npos);
    r.tap(11); r.tap(11); r.advance(60);
    assert(r.robot.output.find("ERR,BUTTON11_ON_REJECTED,BUSY") != std::string::npos);
    r.tap(11);
    const long stoppedA = r.a.currentPosition(), stoppedB = r.b.currentPosition();
    r.advance(2000);
    assert(!r.mechanism.busy() && Wire.lastWritten == 0x0F);
    assert(r.a.currentPosition() == stoppedA && r.b.currentPosition() == stoppedB);
    assert(r.robot.output.find("DONE_POINT_A") == std::string::npos);
    puts("PASS: HOME/OPEN cannot overlap AUTO; single CLOSE cancels work without false DONE");
  }
  {
    Rig r;
    r.tap(11); r.command("STOP"); r.advance(500);
    assert(r.robot.output.find("ACK,BUTTON11,P4_P7_ON_CLOSED") != std::string::npos);
    assert(r.robot.output.find("ACK,BUTTON11,P0_P3_ON_REQUESTED") == std::string::npos);
    assert(Wire.lastWritten == 0xFF);
    r.tap(11);
    setMockDigitalPin(11, LOW); r.advance(29);
    r.robot.feed("STOP\n"); r.advance(5); r.advance(500);
    assert(Wire.lastWritten == 0xFF && !r.mechanism.busy());
    puts("PASS: STOP cancels pending button gestures and prevents a held button from restarting");
  }
  {
    Rig r;
    setMockDigitalPin(17, HIGH); setMockDigitalPin(18, LOW);
    r.command("E18,ARM");
    r.tap(11); r.advance(60); r.tap(11); r.advance(60);
    assert(r.robot.output.find("EVENT,E18_BOTH,1") == std::string::npos);
    // BUTTON11 OFF intentionally disarms E18; arm it again before checking
    // the dedicated GPIO17/GPIO18 inputs.
    r.command("E18,ARM");
    setMockDigitalPin(17, LOW); r.advance(40);
    assert(r.robot.output.find("EVENT,E18_BOTH,1") != std::string::npos);
    assert(r.robot.output.find("E18,STATUS,1,1,1") != std::string::npos);
    puts("PASS: E18 uses GPIO17/18 and ignores the GPIO11 valve button");
  }
  puts("ALL JOG HOST TESTS PASSED (real AccelStepper, simulated clock and GPIO)");
}
