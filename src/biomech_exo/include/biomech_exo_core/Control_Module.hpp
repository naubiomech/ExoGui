#ifndef CONTROL_MODULE_HEADER
#define CONTROL_MODULE_HEADER
#include "Arduino.hpp"
#include "States.hpp"
#include "Control_Algorithms.hpp"
#include "Shaping_Functions.hpp"
#include "Utils.hpp"
#include "Linked_List.hpp"

class ControlModule{
private:
  PID* pid;
  ShapingFunction* shaping_function;
  double clamp_setpoint(double raw_setpoint);
  ControlAlgorithm* current_algorithm;
  bool adjust_shaping_for_time = false;
  Clamp* kf_clamp;
  Clamp* adjust_shaping_for_time_clamp;

  double KF;
  double current_pid_setpoint;
  double pid_input;
  double pid_output;
  double last_control_pid_setpoint;
  RunningAverage* error_average;
  double iter_time_percentage;

  ControlAlgorithm* getControlAlgorithm(StateID state_id);
  double getSetpoint(SensorReport* report);
  double shapeSetpoint(double new_setpoint);
  double runPID(double torque_input, double kf, double pid_setpoint);

public:
  ControlModule(ControlAlgorithm* state_machine, StateID starting_state);
  ~ControlModule();
  void setControlStateMachine(ControlAlgorithm* state_machine, StateID starting_state);
  void setAlgorithmDesiredSetpoint(StateID state, double setpoint);
  double getAlgorithmDesiredSetpoint(StateID state);
  void setToZero();
  void setDesiredSetpoint(double setpoint);
  void changeState(StateID state_id);
  double getControlAdjustment(double torque_input, SensorReport* report);
  void resetStartingParameters();
  void adjustShapingForTime(double planter_time);
  void changeControl(StateID state_id);
  void updateKFPIDError(double torque);
  void applyAutoKF();
  double getLastSetpoint();
  void getPid(double* pid);
  void setPid(double p, double i, double d);
  double getKf();
  void setKf(double kf);
  double getSmoothingParam(StateID state);
  void setSmoothingParam(StateID state, double param);
};

class ControlModuleBuilder{
private:
  LinkedList<StateID> states;
  LinkedList<ControlAlgorithmType> control_types;
public:
  ControlModuleBuilder* addState(StateID state, ControlAlgorithmType control_type);
  ControlModule* build(StateID starting_state);
  ControlAlgorithm* createControlAlgorithm(ControlAlgorithmType type, StateID state_id);
};
#endif
