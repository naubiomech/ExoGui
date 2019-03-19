#ifndef CONTROL_ALGORITHMS_HEADER
#define CONTROL_ALGORITHMS_HEADER
#include "Utils.hpp"
#include "Report.hpp"
#include "States.hpp"
enum ControlAlgorithmType {zero_torque, bang_bang, balance_control, proportional, pivot_proportional};

class ControlAlgorithm{
private:
  Clamp* setpoint_clamp;
  Clamp* activation_clamp;
  int activation_count;
  StateID state_id;
  ControlAlgorithm* next;

  void resetIncrementalActivation();
protected:
  double gain;
  double desired_setpoint;
  double previous_desired_setpoint;
  double used_setpoint;
  int shaping_iteration_threshold;

  double getActivationPercent();
  double clamp_setpoint(double raw_setpoint);
public:
  ControlAlgorithm(StateID state_id);
  virtual ~ControlAlgorithm();
  void setPreviousControlAlgorithm(ControlAlgorithm* control_algorithm);
  void deleteAlgorithmStateMachine();
  virtual void setDesiredSetpoint(double setpoint);
  virtual double getDesiredSetpoint();
  virtual double getShapingIterations();
  virtual void setShapingIterations(double iterations);
  virtual void activate();
  virtual double getSetpoint(SensorReport* report) = 0;
  virtual bool useShapingFunction() = 0;
  virtual ControlAlgorithmType getType() = 0;
  virtual void setToZero();
  virtual void reset();
  virtual void resetStartingParameters();
  virtual void setGain(double gain);
  virtual StateID getStateID();
  virtual ControlAlgorithm* getNextAlgorithm();
};

class ZeroTorqueControl:public ControlAlgorithm{
public:
  ZeroTorqueControl(StateID state_id);
  double getSetpoint(SensorReport* report);
  ControlAlgorithmType getType();
  bool useShapingFunction();
};

class BangBangControl:public ControlAlgorithm{
public:
  BangBangControl(StateID state_id);
  double getSetpoint(SensorReport* report);
  ControlAlgorithmType getType();
  bool useShapingFunction();
  void activate();
};

class BalanceControl:public ControlAlgorithm{
public:
  BalanceControl(StateID state_id);
  double getSetpoint(SensorReport* report);
  ControlAlgorithmType getType();
  double getShapingIterations();
  bool useShapingFunction();
};

class ProportionalControl:public ControlAlgorithm{
public:
  ProportionalControl(StateID state_id);
  double getSetpoint(SensorReport* report);
  ControlAlgorithmType getType();
  bool useShapingFunction();
};

class ProportionalPivotControl:public ControlAlgorithm{
public:
  ProportionalPivotControl(StateID state_id);
  double getSetpoint(SensorReport* report);
  ControlAlgorithmType getType();
  bool useShapingFunction();
};

#endif
