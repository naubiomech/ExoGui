#include "Board.hpp"
#include "Port.hpp"

class BoardDirector{
public:
  virtual Board* build() = 0;
};

class QuadBoardDirector:public BoardDirector{
public:
  Board* build();
};

class BoardBuilder{
private:
  Board* board;
  PortFactory* port_factory;
  unsigned int read_resolution;
  unsigned int write_resolution;
public:
  BoardBuilder(PortFactory* factory);
  Board* build();
  BoardBuilder* reset();
  BoardBuilder* setAnalogReadResolution(unsigned int bits);
  BoardBuilder* setAnalogWriteResolution(unsigned int bits);
  BoardBuilder* setBluetoothTxPort(unsigned int pin);
  BoardBuilder* setBluetoothRxPort(unsigned int pin);
  BoardBuilder* setFsrSenseLeftToePort(unsigned int pin);
  BoardBuilder* setFsrSenseLeftHeelPort(unsigned int pin);
  BoardBuilder* setFsrSenseRightToePort(unsigned int pin);
  BoardBuilder* setFsrSenseRightHeelPort(unsigned int pin);
  BoardBuilder* setTorqueSensorLeftKneePort(unsigned int pin);
  BoardBuilder* setTorqueSensorLeftAnklePort(unsigned int pin);
  BoardBuilder* setTorqueSensorRightKneePort(unsigned int pin);
  BoardBuilder* setTorqueSensorRightAnklePort(unsigned int pin);
  BoardBuilder* setMotorLeftKneePort(unsigned int pin);
  BoardBuilder* setMotorLeftAnklePort(unsigned int pin);
  BoardBuilder* setMotorRightKneePort(unsigned int pin);
  BoardBuilder* setMotorRightAnklePort(unsigned int pin);
  BoardBuilder* setLedPort(unsigned int pin);
  BoardBuilder* setMotorEnablePort(unsigned int pin);
  BoardBuilder* setMotorErrorLeftKneePort(unsigned int pin);
  BoardBuilder* setMotorErrorLeftAnklePort(unsigned int pin);
  BoardBuilder* setMotorErrorRightKneePort(unsigned int pin);
  BoardBuilder* setMotorErrorRightAnklePort(unsigned int pin);
  BoardBuilder* setPotLeftKneePort(unsigned int pin);
  BoardBuilder* setPotRightKneePort(unsigned int pin);
  BoardBuilder* setPotLeftAnklePort(unsigned int pin);
  BoardBuilder* setPotRightAnklePort(unsigned int pin);
  BoardBuilder* setImuSlot0(i2c_pins pins);
  BoardBuilder* setImuSlot1(i2c_pins pins);
  BoardBuilder* setImuSlot2(i2c_pins pins);
  BoardBuilder* setImuAddress0(int address);
  BoardBuilder* setImuAddress1(int address);
};
