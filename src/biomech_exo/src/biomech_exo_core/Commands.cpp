#include "Commands.hpp"
#include "Exoskeleton.hpp"

template <class T>
Command<T>::~Command(){}

void StartTrialCommand::execute(Exoskeleton* exo){
  exo->startTrial();
}

void EndTrialCommand::execute(Exoskeleton* exo){
  exo->endTrial();
}

void CalibrateAllTorquesCommand::execute(Exoskeleton* exo){
  exo->calibrateTorque();
}

void CalibrateAllFsrsCommand::execute(Exoskeleton* exo){
  exo->calibrateFSRs();
}

void SetJointSetpointCommand::execute(Joint* joint){
  joint->setDesiredSetpoint(state, setpoint);
}

void SetJointPidCommand::execute(Joint* joint){
  joint->setPid(p,i,d);
}

void SetJointKfCommand::execute(Joint* joint){
  joint->setKf(kf);
}

void SetJointSmoothingParamCommand::execute(Joint* joint){
  joint->setSmoothingParam(state, param);
}
