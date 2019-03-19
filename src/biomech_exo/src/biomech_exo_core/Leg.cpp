#include "Arduino.hpp"
#include "Leg.hpp"
#include "Shaping_Functions.hpp"
#include "Motor.hpp"
#include "IMU.hpp"
#include "Report.hpp"

Leg::Leg(State* states, LinkedList<Joint*>& joints, LinkedList<FSRGroup*>& fsrs, LinkedList<IMU*>& imus){
  state = states;
  state->setContext(this);

  this->joints = joints;
  this->fsrs = fsrs;
  this->imus = imus;

  this->foot_fsrs = fsrs[0];

  increment_activation_starting_step = 0;
  set_motors_to_zero_torque = false;
  foot_change = new ChangeTrigger(false);
  LegReport* report  = generateReport();
  sensor_report = report->sensor_reports;
  report->sensor_reports = NULL;
  delete report;
}

Leg::~Leg(){
  delete foot_change;
  delete sensor_report;

  state->deleteStateMachine();
  joints.deleteItems();
  fsrs.deleteItems();
  imus.deleteItems();
}

void Leg::attemptCalibration(){
  // TODO add any flag dependent calibrations here
}

void Leg::applyControl(){
  this->applyStateMachine();
  this->adjustControl();
}

void Leg::calibrateFSRs(){
  for (unsigned int i = 0; i < fsrs.size(); i++){
    fsrs[i]->calibrate();
  }
}

void Leg::startTorqueCalibration(){
  for(unsigned int i = 0; i < joints.size();i++){
    joints[i]->startTorqueCalibration();
  }
}

void Leg::updateTorqueCalibration(){
  for(unsigned int i = 0; i < joints.size();i++){
    joints[i]->updateTorqueCalibration();
  }
}

void Leg::endTorqueCalibration(){
  for(unsigned int i = 0; i < joints.size();i++){
    joints[i]->endTorqueCalibration();
  }
}

bool Leg::checkMotorErrors(){
  for(unsigned int i = 0; i < joints.size();i++){
    if(joints[i]->hasErrored()){
      return true;
    }
  }
  return false;
}

void Leg::resetFSRMaxes(){
  for (unsigned int i = 0; i <fsrs.size();i++){
    fsrs[i]->resetMaxes();
  }
}

void Leg::adjustJointSetpoints(){
  fillSensorReport(sensor_report);
  for(unsigned int i = 0; i < joints.size(); i++){
    joints[i]->updateMotorOutput(sensor_report);
  }
}

void Leg::runAutoKF(){
  for(unsigned int i = 0; i < joints.size(); i++){
    joints[i]->applyAutoKF();
  }
}

void Leg::resetStartingParameters(){
  for (unsigned int i = 0; i < joints.size(); i++){
    joints[i]->resetStartingParameters();
  }
}

void Leg::adjustShapingForTime(double time){
  for (unsigned int i =0; i < joints.size(); i++){
    joints[i]->adjustShapingForTime(time);
  }
}

void Leg::setZeroIfSteadyState(){
  if (isSteadyState()){
    setToZero();
  }
}

bool Leg::isSteadyState(){
  return state->getStateTime() > STEADY_STATE_TIMEOUT;
}

void Leg::incrementStepCount(){
  step_count++;
}

void Leg::startIncrementalActivation(){
  this->increment_activation_starting_step = step_count;
}

bool Leg::hasStateChanged(){
  return determine_foot_state_change();
}

void Leg::changeState(){
  state = state->changeState();
  state->setContext(this);
  changeJointControl(state->getStateID());
}

void Leg::adjustControl(){
  this->state->run();
  this->adjustJointSetpoints();
}

void Leg::changeJointControl(StateID state_id){
  for(unsigned int i = 0; i < joints.size();i++){
    joints[i]->changeControl(state_id);
  }
}

bool Leg::determine_foot_state_change(){
  bool foot_on_fsr = foot_fsrs->isActivated();
  bool foot_state_different = foot_change->update(foot_on_fsr);
  return foot_state_different;
}

void Leg::setZeroIfNecessary(){
  if (this->set_motors_to_zero_torque){
    setToZero();
    this->set_motors_to_zero_torque = false;
  }
}

void Leg::setToZero(){
  for(unsigned int i = 0; i<joints.size(); i++){
    joints[i]->setToZero();
  }
}

void Leg::applyStateMachine(){
  bool has_state_changed = this->hasStateChanged();
  if (has_state_changed){
    changeState();
  } else {
    setZeroIfSteadyState();
  }
}

void Leg::measureSensors(){
  for(unsigned int i = 0; i < joints.size(); i++){
    this->joints[i]->measureSensors();
  }

  for (unsigned int i = 0; i < fsrs.size(); i++){
    fsrs[i]->measureForce();
  }

  this->measureIMUs();
}

bool Leg::applyTorque(){
  for(unsigned int i = 0; i < joints.size(); i++){
    if (!joints[i]->applyTorque()){
      return false;
    }
  }
  return true;
}

void Leg::calibrateIMUs(){
  for (unsigned int i = 0; i < imus.size(); i++){
    imus[i]->calibrate();
  }
}

void Leg::measureIMUs(){
  for (unsigned int i = 0; i < imus.size(); i++){
    imus[i]->measure();
  }
}

void Leg::setSign(int sign){
  if (sign == 0){
    return;
  }

  sign = sign / abs(sign);

  for (unsigned int i = 0; i < joints.size(); i++){
    joints[i]->setSign(sign);
  }
}

void Leg::processMessage(LegMessage* msg){
  if (msg == NULL){
    return;
  }
  msg->runPreCommands(this);
  msg->messageJoints(&joints);
  msg->runPostCommands(this);
}

LegReport* Leg::generateReport(){
  LegReport* leg_report = new LegReport();
  SensorReport* sensor_report = new SensorReport();
  leg_report->sensor_reports = sensor_report;
  fillLocalReport(leg_report);
  for (unsigned int i = 0; i < joints.size(); i++){
    leg_report->joint_reports.append(joints[i]->generateReport());
  }
  for (unsigned int i = 0; i < fsrs.size(); i++){
    sensor_report->fsr_reports.append(fsrs[i]->generateReport());
  }
  for (unsigned int i = 0; i < imus.size(); i++){
    sensor_report->imu_reports.append(imus[i]->generateReport());
  }
  return leg_report;
}

void Leg::fillReport(LegReport* report){
  fillLocalReport(report);
  fillSensorReport(report->sensor_reports);
  for (unsigned int i = 0; i < joints.size(); i++){
    joints[i]->fillReport(report->joint_reports[i]);
  }
}

void Leg::fillSensorReport(SensorReport* report){

  for (unsigned int i = 0; i < fsrs.size(); i++){
    fsrs[i]->fillReport(report->fsr_reports[i]);
  }
  for (unsigned int i = 0; i < imus.size(); i++){
    imus[i]->fillReport(report->imu_reports[i]);
  }
}

void Leg::fillLocalReport(LegReport* report){
  report->state = state->getStateType();
}
