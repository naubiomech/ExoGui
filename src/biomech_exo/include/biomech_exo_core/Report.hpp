#ifndef EXO_REPORT_HEADER
#define EXO_REPORT_HEADER
#include <cstddef>
#include "States.hpp"
#include "Linked_List.hpp"

class Report{
public:
  virtual ~Report();
};

class MotorReport:public Report{
public:
  double error;
};

class TorqueSensorReport:public Report{
public:
  double measuredTorque;
};

class PotReport:public Report{
public:
  double angle;
};

class JointReport:public Report{
public:
  JointReport();
  ~JointReport();

  double smoothing[3];
  double pid_setpoint;
  double pid_kf;
  double pid_params[3];
  MotorReport* motor_report;
  TorqueSensorReport* torque_sensor_report;
  PotReport* pot_report;
};

class FSRReport:public Report{
public:
  double threshold;
  double measuredForce;
};

class IMUReport:public Report{
public:
  double orientation[3];
};

class SensorReport:public Report{
public:
  ~SensorReport();
  LinkedList<FSRReport*> fsr_reports;
  LinkedList<IMUReport*> imu_reports;
};

class LegReport:public Report{
public:
  LegReport();
  ~LegReport();
  SensorReport* sensor_reports;
  LinkedList<JointReport*> joint_reports;
  int state;
  StateType phase;
};

class ExoReport:public Report{
public:
  ExoReport();
  ~ExoReport();
  LegReport* left_leg;
  LegReport* right_leg;
};


#endif
