#include "Control_Module.hpp"
#include "Parameters.hpp"
#include <cstddef>

ControlModule::ControlModule(ControlAlgorithm* state_machine, StateID starting_state){

  KF = 1;
  current_pid_setpoint = 0;
  pid_input = 0;
  pid_output = 0;
  iter_time_percentage = 0.5;
  shaping_function = new ShapingFunction();

  setControlStateMachine(state_machine, starting_state);
  this->kf_clamp = new Clamp(MIN_KF, MAX_KF);
  adjust_shaping_for_time_clamp = new Clamp(4, 500);
  this->error_average = new RunningAverage();

  pid = new PID(&this->pid_input, &this->pid_output, &this->current_pid_setpoint,
                PID_DEFAULTS[0], PID_DEFAULTS[1], PID_DEFAULTS[2], REVERSE);
  pid->SetMode(AUTOMATIC);
  pid->SetOutputLimits(-1, 1);
  pid->SetSampleTime(PID_sample_time);
}

ControlModule::~ControlModule(){
  current_algorithm->deleteAlgorithmStateMachine();
  delete pid;
  delete kf_clamp;
  delete adjust_shaping_for_time_clamp;
  delete error_average;
  delete shaping_function;
}

void ControlModule::setAlgorithmDesiredSetpoint(StateID state, double setpoint){
  getControlAlgorithm(state)->setDesiredSetpoint(setpoint);
}

double ControlModule::getAlgorithmDesiredSetpoint(StateID state){
  return getControlAlgorithm(state)->getDesiredSetpoint();
}

double ControlModule::getControlAdjustment(double torque_input, SensorReport* report){
  double setpoint = getSetpoint(report);
  double adjustment = runPID(torque_input, KF, setpoint);
  return adjustment;
}

double ControlModule::getSetpoint(SensorReport* report){
  double new_setpoint = current_algorithm->getSetpoint(report);
  return shapeSetpoint(new_setpoint);
}

double ControlModule::shapeSetpoint(double new_setpoint){
  if (current_algorithm->useShapingFunction()){
    return shaping_function->shape(new_setpoint, current_pid_setpoint, last_control_pid_setpoint);
  }
  return new_setpoint;
}

double ControlModule::runPID(double torque_input, double kf, double pid_setpoint){
  this->pid_input = torque_input;
  this->current_pid_setpoint = pid_setpoint;
  pid->Compute_KF(kf);
  return this->pid_output;
}

ControlAlgorithm* ControlModule::getControlAlgorithm(StateID state_id){
  ControlAlgorithm* alg = current_algorithm;
  do {
    if (alg->getStateID() == state_id){
      return alg;
    }
    alg = alg->getNextAlgorithm();

  }while(alg != current_algorithm);
  return NULL;
}

void ControlModule::setToZero(){
  ControlAlgorithm* alg = current_algorithm;
  while (alg->getNextAlgorithm() != current_algorithm){
    alg->setToZero();
    alg = alg->getNextAlgorithm();
  }
}

void ControlModule::resetStartingParameters(){
  getControlAlgorithm(LATE_STANCE)->setShapingIterations(DEFAULT_ITER_LATE_STANCE);
  getControlAlgorithm(SWING)->setShapingIterations(DEFAULT_ITER_SWING);

  ControlAlgorithm* alg = current_algorithm;
  while (alg->getNextAlgorithm() != current_algorithm){
    alg->resetStartingParameters();
    alg = alg->getNextAlgorithm();
  }
}

void ControlModule::adjustShapingForTime(double planter_time){
  if(adjust_shaping_for_time){
    double iter_late_stance = ((double) ((int) (planter_time + 0.5)) * iter_time_percentage);
    adjust_shaping_for_time_clamp->clamp(iter_late_stance);
    getControlAlgorithm(LATE_STANCE)->setShapingIterations(iter_late_stance);
  }
}

void ControlModule::changeControl(StateID state_id){
  last_control_pid_setpoint = current_pid_setpoint;
  current_algorithm = getControlAlgorithm(state_id);
  current_algorithm->activate();
  shaping_function->setIterationCount(current_algorithm->getShapingIterations());
}

void ControlModule::updateKFPIDError(double torque){
  error_average->update(current_pid_setpoint - torque);
}

void ControlModule::applyAutoKF(){
  double err = error_average->getAverage();
  error_average->reset();

  if (err > max_ERR) {
    KF += 0.05;
  }
  else if (err < min_ERR) {
    KF -= 0.05;
  }

  KF = kf_clamp->clamp(KF);
}

double ControlModule::getLastSetpoint(){
  return current_pid_setpoint;
}

void ControlModule::setControlStateMachine(ControlAlgorithm* state_machine, StateID starting_state){
  current_algorithm = state_machine;
  current_algorithm = getControlAlgorithm(starting_state);
}

void ControlModule::getPid(double* pid){
  pid[0] = this->pid->GetKp();
  pid[1] = this->pid->GetKi();
  pid[2] = this->pid->GetKd();
}

void ControlModule::setPid(double p, double i, double d){
  pid->SetTunings(p,i,d);
}

double ControlModule::getKf(){
  return KF;
}

void ControlModule::setKf(double kf){
  KF = kf;
}

double ControlModule::getSmoothingParam(StateID state){
  return getControlAlgorithm(state)->getShapingIterations();
}

void ControlModule::setSmoothingParam(StateID state, double param){
  getControlAlgorithm(state)->setShapingIterations(param);
}

ControlModuleBuilder* ControlModuleBuilder::addState(StateID state, ControlAlgorithmType control_type){
  states.append(state);
  control_types.append(control_type);
  return this;
}


ControlModule* ControlModuleBuilder::build(StateID starting_state){
  ControlAlgorithm* first = createControlAlgorithm(control_types[0], states[0]);
  ControlAlgorithm* previous = first;
  for(unsigned int i = 1; i < states.size(); i++){
    ControlAlgorithm* alg = createControlAlgorithm(control_types[i], states[i]);
    alg->setPreviousControlAlgorithm(previous);
    previous = alg;
  }
  first->setPreviousControlAlgorithm(previous);
  ControlModule* module = new ControlModule(first, starting_state);
  return module;
}

ControlAlgorithm* ControlModuleBuilder::createControlAlgorithm(ControlAlgorithmType type, StateID state_id){
  switch(type){
  case zero_torque:
    return new ZeroTorqueControl(state_id);
  case bang_bang:
    return new BangBangControl(state_id);
  case balance_control:
    return new BalanceControl(state_id);
  case proportional:
    return new ProportionalControl(state_id);
  case pivot_proportional:
    return new ProportionalPivotControl(state_id);
  default:
    return NULL;
  }
}
