#ifndef EXOSKELETON_HEADER
#define EXOSKELETON_HEADER
#include "Arduino.hpp"
#include "Leg.hpp"
#include "Report.hpp"
#include "Communications.hpp"
#include "Message.hpp"

class Exoskeleton{
private:
  void fillLocalReport(ExoReport* report);
  void disableMotors();
  void enableMotors();
  void attemptCalibration();
  void applyControl();

  bool resetting_motors = false;
  Metro motor_shutdown = Metro(16);
  Metro motor_startup = Metro(8);

  bool trialStarted = false;
  Metro reportDataTimer = Metro(10);
  Metro receiveDataTimer = Metro(1);
  ExoReport* report;
  Communications* comms;
  OutputPort* motor_enable_port;
  OutputPort* led_port;

  Leg* left_leg;
  Leg* right_leg;

public:

  Exoskeleton(Leg* left_leg, Leg* right_leg, Communications* comms,
              OutputPort* motor_enable_port, OutputPort* led_port);
  ~Exoskeleton();
  void run();
  void measureSensors();
  bool checkMotorErrors();
  void disableExo();
  void enableExo();
  void applyTorque();
  void adjustControl();
  void resetStartingParameters();
  void calibrateTorque();
  void calibrateFSRs();
  void setZeroIfStateState();
  void calibrateIMUs();
  void startTrial();
  void endTrial();
  void receiveMessages();
  void checkReset();
  void sendReport();
  void processMessage(ExoMessage* msg);
  ExoReport* generateReport();
  void fillReport(ExoReport* report);
};

#endif
