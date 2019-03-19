#ifndef EXO_COMMAND_HEADER
#define EXO_COMMAND_HEADER
#include "States.hpp"

class Exoskeleton;
class Leg;
class Joint;

template <class T>
class Command{
public:
  virtual ~Command();
  virtual void execute(T* context) = 0;
};

class StartTrialCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class EndTrialCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class CalibrateAllTorquesCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class CalibrateAllFsrsCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class SetJointSetpointCommand:public Command<Joint>{
private:
  StateID state;
  double setpoint;
public:
SetJointSetpointCommand(StateID state, double setpoint): state(state), setpoint(setpoint){};
  virtual void execute(Joint* joint);
};

class SetJointPidCommand:public Command<Joint>{
private:
  double p;
  double i;
  double d;
public:
SetJointPidCommand(double p, double i, double d): p(p), i(i), d(d){};
  virtual void execute(Joint* joint);
};

class SetJointKfCommand:public Command<Joint>{
private:
  double kf;
public:
SetJointKfCommand(double kf): kf(kf){};
  virtual void execute(Joint* joint);
};

class SetJointSmoothingParamCommand:public Command<Joint>{
private:
  StateID state;
  double param;
public:
SetJointSmoothingParamCommand(StateID state, double param): state(state), param(param){};
  virtual void execute(Joint* joint);
};

#endif
