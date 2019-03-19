#include "Joint.hpp"
#include "Board.hpp"
#include "Message.hpp"

Joint::Joint(ControlModule* controller, Motor* motor, TorqueSensor* torque_sensor, Pot* pot_sensor){
  this->controller = controller;
  this->motor = motor;
  this->torque_sensor = torque_sensor;
  this->pot = pot_sensor;

  motor_output = 0;
}

Joint::~Joint(){
  delete controller;
  delete motor;
  delete torque_sensor;
  delete pot;
}

void Joint::setDesiredSetpoint(StateID state, double setpoint){
  controller->setAlgorithmDesiredSetpoint(state, setpoint);
}

double Joint::getDesiredSetpoint(StateID state){
  return controller->getAlgorithmDesiredSetpoint(state);
}

void Joint::measureError(){
  motor->measureError();
}

bool Joint::hasErrored(){
  return motor->hasErrored();
}

bool Joint::applyTorque(){
  if (torque_sensor->getTorque() > 25){
    return false;
  }
  motor->write(motor_output);
  return true;
}

void Joint::changeControl(StateID state_id){
  controller->changeControl(state_id);
}

void Joint::updateMotorOutput(SensorReport* report){
  double torque = torque_sensor->getTorque();
  motor_output = controller->getControlAdjustment(torque, report);
}

void Joint::setToZero(){
  controller->setToZero();
}

void Joint::resetStartingParameters(){
  controller->resetStartingParameters();
}

void Joint::adjustShapingForTime(double planter_time){
  this->controller->adjustShapingForTime(planter_time);
}

void Joint::applyAutoKF(){
  controller->applyAutoKF();
}

void Joint::measureSensors(){
  this->measureError();
  this->measureTorque();
  this->measurePot();
}

void Joint::measureTorque(){
  torque_sensor->measureTorque();
}

void Joint::measurePot(){
  if (this->pot != NULL){
    this->pot->measure();
  }
}

void Joint::startTorqueCalibration(){
  void setTorqueScalar(double scalar);
  torque_sensor->startTorqueCalibration();
}

void Joint::updateTorqueCalibration(){
  torque_sensor->updateTorqueCalibration();
}

void Joint::endTorqueCalibration(){
  torque_sensor->endTorqueCalibration();
}

void Joint::setSign(int sign){
  motor->setSign(sign);
  torque_sensor->setSign(sign);
}

void Joint::setPid(double p, double i, double d){
  controller->setPid(p,i,d);
}

void Joint::getPid(double* pid){
  controller->getPid(pid);
}

void Joint::processMessage(JointMessage* msg){
  msg->runCommands(this);
}

double Joint::getKf(){
  return controller->getKf();
}

void Joint::setKf(double kf){
  controller->setKf(kf);
}

double Joint::getSmoothingParam(StateID state){
  return controller->getSmoothingParam(state);
}

void Joint::setSmoothingParam(StateID state, double param){
  controller->setSmoothingParam(state, param);
}

double Joint::getAngle(){
  if (this->pot == NULL){
    return -1;
  }
  return this->pot->getAngle();
}

JointReport* Joint::generateReport(){
  JointReport* report = new JointReport();
  fillLocalReport(report);
  report->motor_report = motor->generateReport();
  report->torque_sensor_report = torque_sensor->generateReport();

  if (this->pot == NULL){
    report->pot_report = NULL;
  } else {
    report->pot_report = this->pot->generateReport();
  }

  return report;
}

void Joint::fillReport(JointReport* report){
  fillLocalReport(report);
  motor->fillReport(report->motor_report);
  torque_sensor->fillReport(report->torque_sensor_report);

  if (this->pot != NULL){
    this->pot->fillReport(report->pot_report);
  }

}

void Joint::fillLocalReport(JointReport* report){
  getPid(report->pid_params);
  report->pid_kf = getKf();
  report->smoothing[0] = getSmoothingParam(SWING);
  report->smoothing[1] = 0;
  report->smoothing[2] = getSmoothingParam(LATE_STANCE);
  report->pid_setpoint = getDesiredSetpoint(LATE_STANCE);
}
