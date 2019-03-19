#ifndef JOINT_HEADER
#define JOINT_HEADER

#include "Motor.hpp"
#include "TorqueSensor.hpp"
#include "Pot.hpp"
#include "Report.hpp"
#include "Port.hpp"
#include "Control_Module.hpp"
#include "Message.hpp"

class Joint{
private:
  Motor* motor;
  TorqueSensor* torque_sensor;
  Pot* pot;
  ControlModule* controller;

  double motor_output;

  void fillLocalReport(JointReport* report);
  void measureError();
  void measureTorque();
  void measurePot();
public:
  Joint(ControlModule* controller, Motor* motor, TorqueSensor* torque_sensor, Pot* pot);
  ~Joint();
  void processMessage(JointMessage* msg);
  JointReport* generateReport();
  void fillReport(JointReport* report);

  void setDesiredSetpoint(StateID state, double setpoint);
  double getDesiredSetpoint(StateID state);
  void measureSensors();
  double getAngle();
  bool hasErrored();
  bool applyTorque();
  void setToZero();
  void resetStartingParameters();
  void adjustShapingForTime(double planterTime);
  void updateMotorOutput(SensorReport* report);
  void changeControl(StateID state_id);
  void applyAutoKF();
  double getTorque();
  void setSign(int sign);
  void setPid(double p, double i, double d);
  void getPid(double* pid);
  double getKf();
  void setKf(double kf);
  double getSmoothingParam(StateID state);
  void setSmoothingParam(StateID state, double param);
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
};

#endif
