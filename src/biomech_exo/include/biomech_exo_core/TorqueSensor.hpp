#ifndef TORQUE_SENSOR_HEADER
#define TORQUE_SENSOR_HEADER

#include "Report.hpp"
#include "Port.hpp"
#include "Utils.hpp"

class TorqueSensor{
private:
  InputPort* torque_sensor_port;
  MovingAverage* torque_averager;
  RunningAverage* torque_calibration_average;
  double torque_calibration_value;
  int torque_address;
  double torque_sign;

  double measureRawTorque();
  double measureRawCalibratedTorque();
  void fillLocalReport(TorqueSensorReport* report);
public:
  TorqueSensor(InputPort* torque_sensor_port, int sign);
  ~TorqueSensor();
  double getTorque();
  void measureTorque();
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
  void setSign(int sign);
  TorqueSensorReport* generateReport();
  void fillReport(TorqueSensorReport* report);
};

#endif
