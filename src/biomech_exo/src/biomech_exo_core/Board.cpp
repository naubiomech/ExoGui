#include "Arduino.hpp"
#include "Board.hpp"
#include "Port.hpp"
#include "Linked_List.hpp"

Board::Board(){
  bluetooth_tx_port = NULL;
  bluetooth_rx_port = NULL;
  fsr_sense_left_toe_port = NULL;
  fsr_sense_left_heel_port = NULL;
  fsr_sense_right_toe_port = NULL;
  fsr_sense_right_heel_port = NULL;
  torque_sensor_left_knee_port = NULL;
  torque_sensor_left_ankle_port = NULL;
  torque_sensor_right_knee_port = NULL;
  torque_sensor_right_ankle_port = NULL;
  motor_left_knee_port = NULL;
  motor_left_ankle_port = NULL;
  motor_right_knee_port = NULL;
  motor_right_ankle_port = NULL;
  led_port = NULL;
  motor_enable_port = NULL;
  motor_error_left_knee_port = NULL;
  motor_error_left_ankle_port = NULL;
  motor_error_right_knee_port = NULL;
  motor_error_right_ankle_port = NULL;
  imu_slot_0 = NULL;
  imu_slot_1 = NULL;
  imu_slot_2 = NULL;
  pot_left_knee_port = NULL;
  pot_right_knee_port = NULL;
  pot_left_ankle_port = NULL;
  pot_right_ankle_port = NULL;
}

Board::~Board(){
  delete bluetooth_tx_port;
  delete bluetooth_rx_port;
  delete fsr_sense_left_toe_port;
  delete fsr_sense_left_heel_port;
  delete fsr_sense_right_toe_port;
  delete fsr_sense_right_heel_port;
  delete torque_sensor_left_knee_port;
  delete torque_sensor_left_ankle_port;
  delete torque_sensor_right_knee_port;
  delete torque_sensor_right_ankle_port;
  delete motor_left_knee_port;
  delete motor_left_ankle_port;
  delete motor_right_knee_port;
  delete motor_right_ankle_port;
  delete led_port;
  delete motor_enable_port;
  delete motor_error_left_knee_port;
  delete motor_error_left_ankle_port;
  delete motor_error_right_knee_port;
  delete motor_error_right_ankle_port;
  delete imu_slot_0;
  delete imu_slot_1;
  delete imu_slot_2;
  delete pot_left_knee_port;
  delete pot_right_knee_port;
  delete pot_left_ankle_port;
  delete pot_right_ankle_port;
}

void Board::turnOnLed(){
  if (led_port){
    led_port->write(1);
  }
}

TxPort* Board::takeBluetoothTxPort(){
  TxPort* port = bluetooth_tx_port;
  bluetooth_tx_port = NULL;
  return port;
}

RxPort* Board::takeBluetoothRxPort(){
  RxPort* port = bluetooth_rx_port;
  bluetooth_rx_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseLeftToePort(){
  InputPort* port = fsr_sense_left_toe_port;
  fsr_sense_left_toe_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseLeftHeelPort(){
  InputPort* port = fsr_sense_left_heel_port;
  fsr_sense_left_heel_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseRightToePort(){
  InputPort* port = fsr_sense_right_toe_port;
  fsr_sense_right_toe_port = NULL;
  return port;
}

InputPort* Board::takeFsrSenseRightHeelPort(){
  InputPort* port = fsr_sense_right_heel_port;
  fsr_sense_right_heel_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorLeftKneePort(){
  InputPort* port = torque_sensor_left_knee_port;
  torque_sensor_left_knee_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorLeftAnklePort(){
  InputPort* port = torque_sensor_left_ankle_port;
  torque_sensor_left_ankle_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorRightKneePort(){
  InputPort* port = torque_sensor_right_knee_port;
  torque_sensor_right_knee_port = NULL;
  return port;
}

InputPort* Board::takeTorqueSensorRightAnklePort(){
  InputPort* port = torque_sensor_right_ankle_port;
  torque_sensor_right_ankle_port = NULL;
  return port;
}

OutputPort* Board::takeMotorLeftKneePort(){
  OutputPort* port = motor_left_knee_port;
  motor_left_knee_port = NULL;
  return port;
}

OutputPort* Board::takeMotorLeftAnklePort(){
  OutputPort* port = motor_left_ankle_port;
  motor_left_ankle_port = NULL;
  return port;
}

OutputPort* Board::takeMotorRightKneePort(){
  OutputPort* port = motor_right_knee_port;
  motor_right_knee_port = NULL;
  return port;
}

OutputPort* Board::takeMotorRightAnklePort(){
  OutputPort* port = motor_right_ankle_port;
  motor_right_ankle_port = NULL;
  return port;
}

OutputPort* Board::takeLedPort(){
  OutputPort* port = led_port;
  led_port = NULL;
  return port;
}

OutputPort* Board::takeMotorEnablePort(){
  OutputPort* port = motor_enable_port;
  motor_enable_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorLeftKneePort(){
  InputPort* port = motor_error_left_knee_port;
  motor_error_left_knee_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorLeftAnklePort(){
  InputPort* port = motor_error_left_ankle_port;
  motor_error_left_ankle_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorRightKneePort(){
  InputPort* port = motor_error_right_knee_port;
  motor_error_right_knee_port = NULL;
  return port;
}

InputPort* Board::takeMotorErrorRightAnklePort(){
  InputPort* port = motor_error_right_ankle_port;
  motor_error_right_ankle_port = NULL;
  return port;
}

InputPort* Board::takePotLeftKneePort(){
  InputPort* port = pot_left_knee_port;
  pot_left_knee_port = NULL;
  return port;
}

InputPort* Board::takePotRightKneePort(){
  InputPort* port = pot_right_knee_port;
  pot_right_knee_port = NULL;
  return port;
}

InputPort* Board::takePotLeftAnklePort(){
  InputPort* port = pot_left_ankle_port;
  pot_left_ankle_port = NULL;
  return port;
}

InputPort* Board::takePotRightAnklePort(){
  InputPort* port = pot_right_ankle_port;
  pot_right_ankle_port = NULL;
  return port;
}

void Board::setBluetoothTxPort(TxPort* port){
  bluetooth_tx_port = port;
}

void Board::setBluetoothRxPort(RxPort* port){
  bluetooth_rx_port = port;
}

void Board::setFsrSenseLeftToePort(InputPort* port){
  fsr_sense_left_toe_port = port;
}

void Board::setFsrSenseLeftHeelPort(InputPort* port){
  fsr_sense_left_heel_port = port;
}

void Board::setFsrSenseRightToePort(InputPort* port){
  fsr_sense_right_toe_port = port;
}

void Board::setFsrSenseRightHeelPort(InputPort* port){
  fsr_sense_right_heel_port = port;
}

void Board::setTorqueSensorLeftKneePort(InputPort* port){
  torque_sensor_left_knee_port = port;
}

void Board::setTorqueSensorLeftAnklePort(InputPort* port){
  torque_sensor_left_ankle_port = port;
}

void Board::setTorqueSensorRightKneePort(InputPort* port){
  torque_sensor_right_knee_port = port;
}

void Board::setTorqueSensorRightAnklePort(InputPort* port){
  torque_sensor_right_ankle_port = port;
}

void Board::setMotorLeftKneePort(OutputPort* port){
  motor_left_knee_port = port;
}

void Board::setMotorLeftAnklePort(OutputPort* port){
  motor_left_ankle_port = port;
}

void Board::setMotorRightKneePort(OutputPort* port){
  motor_right_knee_port = port;
}

void Board::setMotorRightAnklePort(OutputPort* port){
  motor_right_ankle_port = port;
}

void Board::setLedPort(OutputPort* port){
  led_port = port;
}

void Board::setMotorEnablePort(OutputPort* port){
  motor_enable_port = port;
}

void Board::setMotorErrorLeftKneePort(InputPort* port){
  motor_error_left_knee_port = port;
}

void Board::setMotorErrorLeftAnklePort(InputPort* port){
  motor_error_left_ankle_port = port;
}

void Board::setMotorErrorRightKneePort(InputPort* port){
  motor_error_right_knee_port = port;
}

void Board::setMotorErrorRightAnklePort(InputPort* port){
  motor_error_right_ankle_port = port;
}

void Board::setImuSlot0(ImuPort* port){
  imu_slot_0 = port;
}

void Board::setImuSlot1(ImuPort* port){
  imu_slot_1 = port;
}

void Board::setImuSlot2(ImuPort* port){
  imu_slot_2 = port;
}

void Board::setImuAddress0(int address){
  imu_address_0 = address;
}

void Board::setImuAddress1(int address){
  imu_address_1 = address;
}

void Board::setPotLeftKneePort(InputPort* port){
  pot_left_knee_port = port;
}

void Board::setPotRightKneePort(InputPort* port){
  pot_right_knee_port = port;
}

void Board::setPotLeftAnklePort(InputPort* port){
  pot_left_ankle_port = port;
}

void Board::setPotRightAnklePort(InputPort* port){
  pot_right_ankle_port = port;
}

ImuPort* Board::getImuSlot0(){
  return imu_slot_0;
}

ImuPort* Board::getImuSlot1(){
  return imu_slot_1;
}

ImuPort* Board::getImuSlot2(){
  return imu_slot_2;
}

int Board::getImuAddress0(){
  return imu_address_0;
}

int Board::getImuAddress1(){
  return imu_address_1;
}
