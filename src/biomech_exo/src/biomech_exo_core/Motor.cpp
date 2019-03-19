#include "Arduino.hpp"
#include "Motor.hpp"
#include "Parameters.hpp"
#include "Board.hpp"
#include "Utils.hpp"
#include "States.hpp"
#include "Control_Algorithms.hpp"
#include "Report.hpp"

Motor::Motor(InputPort* motor_error_port, OutputPort* motor_port, int output_sign){
  setSign(output_sign);
  this->motor_port = motor_port;
  this->motor_error_port = motor_error_port;
  this->zero_offset = MOTOR_ZERO_OFFSET_DEFAULT;
}

Motor::~Motor(){
  delete motor_port;
  delete motor_error_port;
}

void Motor::write(double motor_output){
  double voltage = (output_sign * motor_output) + this->zero_offset;
  voltage = (voltage + 1.0)/2.0;

  this->motor_port->write(voltage);
}

void Motor::measureError(){
  in_error_state = motor_error_port->read();
}

bool Motor::hasErrored(){
  return in_error_state;
}

void Motor::setSign(int sign){
  this->output_sign = (double) sign;
}

MotorReport* Motor::generateReport(){
  MotorReport* report = new MotorReport();
  fillLocalReport(report);
  return report;
}

void Motor::fillReport(MotorReport* report){
  fillLocalReport(report);
}

void Motor::fillLocalReport(MotorReport* report){
  report->error = hasErrored();
}
