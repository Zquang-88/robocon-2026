#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include "UartProtocol.h"
#include "Wire.h"

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
  Rig() { resetMockDigitalPins(); Wire.reset(); loadDefaultMechanismConfig(config); mechanism.begin(config); protocol.begin(); advance(30); }
  void command(const std::string &line) { robot.feed(line + "\n"); protocol.update(); }
  void advance(uint32_t ms) {
    const auto until = mockMicros + uint64_t(ms) * 1000;
    while (mockMicros < until) { mockMicros += 100; protocol.update(); }
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
    for (int i = 0; i < 40000 &&
                    r.robot.output.find("DONE_POINT_B") == std::string::npos;
         ++i) r.advance(1);
    assert(r.robot.output.find("DONE_POINT_B") != std::string::npos);
    assert(r.mechanism.busy());
    assert(strcmp(r.mechanism.activeProfileName(), "PICK_B") == 0);
    r.advance(20000);
    assert(!r.mechanism.busy());
    assert(r.robot.output.find("DONE,PICK_B") == std::string::npos);
    puts("PASS: POINT_B releases chassis while mechanism retracts in background");
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
    assert(r.robot.output.find("[POINT_A] P2 ON, P5 OFF") != std::string::npos);
    assert(r.robot.output.find("[POINT_A] P3 ON, P4 OFF") != std::string::npos);
    assert(r.robot.output.find("DONE_POINT_A") != std::string::npos);
    assert(Wire.lastWritten == 0xF3);
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
    puts("PASS: THA_2B P2 OFF then P0 OFF with a verified 350 ms interval");
  }
  {
    Rig r;
    r.command("POINT_B");
    for (int i = 0; i < 50000 &&
                    r.robot.output.find("DONE_POINT_B") == std::string::npos;
         ++i) r.advance(1);
    assert(Wire.lastWritten == 0xFC);  // P0/P1 LOW; all others HIGH
    assert(r.robot.output.find("[POINT_B] P0 ON, P7 OFF") != std::string::npos);
    assert(r.robot.output.find("[POINT_B] P1 ON, P6 OFF") != std::string::npos);
    puts("PASS: POINT_B exact pair order, bits and response");
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
    assert(Wire.lastWritten == 0xF3);  // P2/P3 retained ON
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
  puts("ALL JOG HOST TESTS PASSED (real AccelStepper, simulated clock and GPIO)");
}
