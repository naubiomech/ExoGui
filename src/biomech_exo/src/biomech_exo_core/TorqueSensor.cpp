#include "Arduino.hpp"
#include "TorqueSensor.hpp"
#include "Port.hpp"
#include "Report.hpp"
#include "Parameters.hpp"

TorqueSensor::TorqueSensor(InputPort* torque_sensor_port, int sign){
  torque_sign = 1.0;
  torque_calibration_value = 0;
  this->torque_sensor_port = torque_sensor_port;
  this->torque_averager = new MovingAverage(TORQUE_AVERAGE_COUNT);
  this->torque_calibration_average = new RunningAverage();
  setSign(sign);
}

TorqueSensor::~TorqueSensor(){
  delete torque_sensor_port;
  delete torque_averager;
  delete torque_calibration_average;
}

void TorqueSensor::measureTorque(){
  double measured = this->measureRawCalibratedTorque();
  torque_averager->update(measured);
}

double TorqueSensor::getTorque(){
  double torque = torque_averager->getAverage();
  return torque;
}

void TorqueSensor::startTorqueCalibration(){
  torque_calibration_average->reset();
}

void TorqueSensor::updateTorqueCalibration(){
  torque_calibration_average->update(measureRawTorque());
}

void TorqueSensor::endTorqueCalibration(){
  torque_calibration_value = torque_calibration_average->getAverage();
}

double TorqueSensor::measureRawTorque(){
  double readValue = this->torque_sensor_port->read();
  double adjustVal = readValue * 2 - 1;
  return adjustVal * torque_sign * VOLTAGE_TO_TORQUE_CONVERSION;
}

double TorqueSensor::measureRawCalibratedTorque(){
  double rawTorq = measureRawTorque();
  double Torq = rawTorq - this->torque_calibration_value;
  return Torq;
}

TorqueSensorReport* TorqueSensor::generateReport(){
  TorqueSensorReport* report = new TorqueSensorReport();
  fillLocalReport(report);
  return report;
}

void TorqueSensor::setSign(int sign){
  torque_sign = (double) sign;
}

void TorqueSensor::fillReport(TorqueSensorReport* report){
  fillLocalReport(report);
}

void TorqueSensor::fillLocalReport(TorqueSensorReport* report){
  report->measuredTorque = getTorque();
}
