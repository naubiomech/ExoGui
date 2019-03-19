#include "Arduino.hpp"
#include "Exoskeleton.hpp"
#include "Board.hpp"
#include "Leg.hpp"
#include "Report.hpp"
#include "Communications.hpp"
#include "Message.hpp"

Exoskeleton::Exoskeleton(Leg* left_leg, Leg* right_leg, Communications* comms,
                         OutputPort* motor_enable_port, OutputPort* led_port){
  this->left_leg = left_leg;
  this->right_leg = right_leg;
  this->led_port = led_port;
  this->motor_enable_port = motor_enable_port;
  this->comms = comms;

  motor_enable_port->write(0);
  report = generateReport();
  led_port->write(0);

  this->calibrateTorque();
}

Exoskeleton::~Exoskeleton(){
  delete left_leg;
  delete right_leg;

  delete motor_enable_port;
  delete led_port;
  delete report;
  delete comms;
}

void Exoskeleton::startTrial(){
  enableExo();
  trialStarted = true;
  reportDataTimer.reset();
  receiveDataTimer.reset();
}

void Exoskeleton::endTrial(){
  disableExo();
  trialStarted = false;
}

void Exoskeleton::run(){
  this->measureSensors();
  //TODO implement error checking
  this->attemptCalibration();
  this->applyControl();
  this->applyTorque();
}

void Exoskeleton::sendReport(){
  if (trialStarted &&
      reportDataTimer.check()) {

    reportDataTimer.reset();
    fillReport(report);
    comms->sendReport(report);
  }
}

void Exoskeleton::attemptCalibration(){
  left_leg->attemptCalibration();
  right_leg->attemptCalibration();
}

void Exoskeleton::applyControl(){
  left_leg->applyControl();
  right_leg->applyControl();
}

void Exoskeleton::calibrateTorque(){
  left_leg->startTorqueCalibration();
  right_leg->startTorqueCalibration();
  long torque_calibration_value_time = millis();
  while (millis() - torque_calibration_value_time < 1000){
    left_leg->updateTorqueCalibration();
    right_leg->updateTorqueCalibration();
  }
  left_leg->endTorqueCalibration();
  right_leg->endTorqueCalibration();
}

void Exoskeleton::calibrateFSRs(){
  left_leg->calibrateFSRs();
  right_leg->calibrateFSRs();
}

void Exoskeleton::calibrateIMUs(){
  right_leg->calibrateIMUs();
  left_leg->calibrateIMUs();
}

void Exoskeleton::resetStartingParameters(){
  left_leg->resetStartingParameters();
  right_leg->resetStartingParameters();
}

void Exoskeleton::measureSensors(){
  left_leg->measureSensors();
  right_leg->measureSensors();
}

bool Exoskeleton::checkMotorErrors(){
  if (!resetting_motors && (left_leg->checkMotorErrors() || right_leg->checkMotorErrors())){
    disableMotors();
    resetting_motors = true;
    motor_shutdown.reset();
  }

  if (resetting_motors && motor_shutdown.check()){
    enableMotors();
    motor_startup.reset();
  }

  if (resetting_motors && motor_startup.check()){
    resetting_motors = false;
  }

  return resetting_motors;
}

void Exoskeleton::disableMotors(){
  motor_enable_port->write(0);
}

void Exoskeleton::enableMotors(){
  motor_enable_port->write(1);
}

void Exoskeleton::enableExo(){
  enableMotors();
  led_port->write(1);
}

void Exoskeleton::disableExo(){
  disableMotors();
  led_port->write(0);
}

void Exoskeleton::applyTorque(){
  if(!(left_leg->applyTorque() &&
       right_leg->applyTorque())){
    disableExo();
  }
}

void Exoskeleton::receiveMessages(){
  if (receiveDataTimer.check() == 1) {
    receiveDataTimer.reset();
    fillReport(report);
    ExoMessage* msg = comms->receiveMessages(report);
    processMessage(msg);
    delete msg;
  }
}

void Exoskeleton::processMessage(ExoMessage* msg){
  msg->runPreCommands(this);
  msg->messageLeftLeg(left_leg);
  msg->messageRightLeg(right_leg);
  msg->runPostCommands(this);
}

void Exoskeleton::checkReset(){
  if (!trialStarted) {
    resetStartingParameters();
  }
}

ExoReport* Exoskeleton::generateReport(){
  ExoReport* report = new ExoReport();
  fillLocalReport(report);
  report->left_leg = left_leg->generateReport();
  report->right_leg = right_leg->generateReport();
  return report;
}

void Exoskeleton::fillReport(ExoReport* report){
  fillLocalReport(report);
  left_leg->fillReport(report->left_leg);
  right_leg->fillReport(report->right_leg);
}

void Exoskeleton::fillLocalReport(ExoReport*){

}
