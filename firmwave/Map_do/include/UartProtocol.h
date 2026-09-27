#pragma once

#include <Arduino.h>

#include "MechanismController.h"
#include "LocalButton.h"
#include "PersistentConfig.h"

class UartProtocol {
 public:
  UartProtocol(Stream &usb, Stream &robot,
               MechanismController &controller,
               MechanismConfig &config,
               PersistentConfig &persistentConfig);
  void begin();
  void update();

 private:
  struct Receiver { char buffer[256] = {}; size_t length = 0; bool dropping = false; };
  Stream &usb_;
  Stream &robot_;
  MechanismController &controller_;
  MechanismConfig &config_;
  PersistentConfig &persistentConfig_;
  Receiver usbReceiver_;
  Receiver robotReceiver_;
  uint32_t lastStatusMs_ = 0;
  float manualSpeed_ = 5000.0f;
  bool e18Armed_ = false;
  bool e18Latched_ = false;
  uint32_t e18BothSinceMs_ = 0;
  bool readyQueued_ = false;
  bool pointAComplete_ = false;
  bool pointBComplete_ = false;
  bool pointCComplete_ = false;
  bool bridgeBComplete_ = false;
  bool drop2AComplete_ = false;
  bool drop2BComplete_ = false;
  LocalButton valveButton_;
  LocalButton homeButton_;
  bool valveClickPending_ = false;
  bool valveDoubleClickCanEnable_ = false;
  uint32_t valveFirstClickMs_ = 0;

  void readStream(Stream &stream, Receiver &receiver);
  void processLine(char *line);
  void emit(const char *line);
  void emitAck(const char *token);
  void emitError(const char *code, const char *detail = nullptr);
  void emitStatus();
  void emitConfig();
  void emitProfile(ProfileId id);
  void emitCapabilities();
  void emitE18Status();
  void emitHomeStatus();
  void emitFieldStatus();
  void updateE18();
  void updateLocalButtons();
  void cancelButtonGestures();
  void closeValvesFromButton();
  void processJog(const char *arguments);
  bool runNamedProfile(const char *name);
  bool startCompetitionCommand(const char *wireCommand,
                               CompetitionAction action);
  void resetCompetitionLatches();
  bool updateProfile(const char *arguments);
};
