#ifndef BOARD_SETTINGS_HEADER
#define BOARD_SETTINGS_HEADER
#include "Parameters.hpp"
#include "Exoskeleton.hpp"
#include "Port.hpp"

class Board{
private:
  TxPort* bluetooth_tx_port;
  RxPort* bluetooth_rx_port;
  InputPort* fsr_sense_left_toe_port;
  InputPort* fsr_sense_left_heel_port;
  InputPort* fsr_sense_right_toe_port;
  InputPort* fsr_sense_right_heel_port;
  InputPort* torque_sensor_left_knee_port;
  InputPort* torque_sensor_left_ankle_port;
  InputPort* torque_sensor_right_knee_port;
  InputPort* torque_sensor_right_ankle_port;
  OutputPort* motor_left_knee_port;
  OutputPort* motor_left_ankle_port;
  OutputPort* motor_right_knee_port;
  OutputPort* motor_right_ankle_port;
  OutputPort* led_port;
  OutputPort* motor_enable_port;
  InputPort* motor_error_left_knee_port;
  InputPort* motor_error_left_ankle_port;
  InputPort* motor_error_right_knee_port;
  InputPort* motor_error_right_ankle_port;
  InputPort* pot_left_leg_port;
  InputPort* pot_right_leg_port;
  InputPort* pot_left_knee_port;
  InputPort* pot_right_knee_port;
  InputPort* pot_left_ankle_port;
  InputPort* pot_right_ankle_port;
  ImuPort* imu_slot_0;
  ImuPort* imu_slot_1;
  ImuPort* imu_slot_2;
  int imu_address_0;
  int imu_address_1;

public:
  Board();
  ~Board();

  void turnOnLed();
  TxPort* takeBluetoothTxPort();
  RxPort* takeBluetoothRxPort();
  InputPort* takeFsrSenseLeftToePort();
  InputPort* takeFsrSenseLeftHeelPort();
  InputPort* takeFsrSenseRightToePort();
  InputPort* takeFsrSenseRightHeelPort();
  InputPort* takeTorqueSensorLeftKneePort();
  InputPort* takeTorqueSensorLeftAnklePort();
  InputPort* takeTorqueSensorRightKneePort();
  InputPort* takeTorqueSensorRightAnklePort();
  OutputPort* takeMotorLeftKneePort();
  OutputPort* takeMotorLeftAnklePort();
  OutputPort* takeMotorRightKneePort();
  OutputPort* takeMotorRightAnklePort();
  OutputPort* takeLedPort();
  OutputPort* takeMotorEnablePort();
  InputPort* takeMotorErrorLeftKneePort();
  InputPort* takeMotorErrorLeftAnklePort();
  InputPort* takeMotorErrorRightKneePort();
  InputPort* takeMotorErrorRightAnklePort();
  InputPort* takePotLeftKneePort();
  InputPort* takePotRightKneePort();
  InputPort* takePotLeftAnklePort();
  InputPort* takePotRightAnklePort();
  ImuPort* getImuSlot0();
  ImuPort* getImuSlot1();
  ImuPort* getImuSlot2();
  int getImuAddress0();
  int getImuAddress1();

  void setBluetoothTxPort(TxPort* port);
  void setBluetoothRxPort(RxPort* port);
  void setFsrSenseLeftToePort(InputPort* port);
  void setFsrSenseLeftHeelPort(InputPort* port);
  void setFsrSenseRightToePort(InputPort* port);
  void setFsrSenseRightHeelPort(InputPort* port);
  void setTorqueSensorLeftKneePort(InputPort* port);
  void setTorqueSensorLeftAnklePort(InputPort* port);
  void setTorqueSensorRightKneePort(InputPort* port);
  void setTorqueSensorRightAnklePort(InputPort* port);
  void setMotorLeftKneePort(OutputPort* port);
  void setMotorLeftAnklePort(OutputPort* port);
  void setMotorRightKneePort(OutputPort* port);
  void setMotorRightAnklePort(OutputPort* port);
  void setLedPort(OutputPort* port);
  void setMotorEnablePort(OutputPort* port);
  void setMotorErrorLeftKneePort(InputPort* port);
  void setMotorErrorLeftAnklePort(InputPort* port);
  void setMotorErrorRightKneePort(InputPort* port);
  void setMotorErrorRightAnklePort(InputPort* port);
  void setPotLeftKneePort(InputPort* port);
  void setPotRightKneePort(InputPort* port);
  void setPotLeftAnklePort(InputPort* port);
  void setPotRightAnklePort(InputPort* port);
  void setImuSlot0(ImuPort* port);
  void setImuSlot1(ImuPort* port);
  void setImuSlot2(ImuPort* port);
  void setImuAddress0(int address);
  void setImuAddress1(int address);
};

Exoskeleton* setupBoard();
#endif
