#include "BoardBuilder.hpp"

Board* QuadBoardDirector::build(){
  PortFactory* portFactory = new ArduinoPortFactory();
  BoardBuilder* builder = new BoardBuilder(portFactory);
  Board* board = builder
    ->setAnalogWriteResolution(12)
    ->setAnalogReadResolution(12)
    ->setBluetoothTxPort(0)
    ->setBluetoothRxPort(1)
    ->setLedPort(13)
    ->setFsrSenseRightHeelPort(A12)
    ->setFsrSenseRightToePort(A13)
    ->setFsrSenseLeftHeelPort(A14)
    ->setFsrSenseLeftToePort(A15)
    ->setTorqueSensorRightAnklePort(A0)
    ->setTorqueSensorRightKneePort(A1)
    ->setTorqueSensorLeftAnklePort(A6)
    ->setTorqueSensorLeftKneePort(A5)
    ->setMotorLeftKneePort(23)
    ->setMotorLeftAnklePort(22)
    ->setMotorRightKneePort(5)
    ->setMotorRightAnklePort(6)
    ->setMotorEnablePort(17)
    ->setMotorErrorLeftKneePort(24)
    ->setMotorErrorLeftAnklePort(25)
    ->setMotorErrorRightKneePort(26)
    ->setMotorErrorRightAnklePort(27)
    ->setPotRightAnklePort(A2)
    ->setPotLeftAnklePort(A16)
    ->setImuSlot0(I2C_PINS_7_8)
    ->setImuSlot2(I2C_PINS_3_4)
    ->setImuAddress0(0x28)
    ->setImuAddress1(0x29)
    ->build();

  delete builder;
  delete portFactory;
  board->turnOnLed();
  return board;
}

BoardBuilder::BoardBuilder(PortFactory* factory){
  port_factory = factory;
  reset();
}

Board* BoardBuilder::build(){

  return board;
}

BoardBuilder* BoardBuilder::reset(){
  read_resolution = 10;
  write_resolution = 10;
  board = new Board();
  return this;
}

BoardBuilder* BoardBuilder::setAnalogWriteResolution(unsigned int bits){
  write_resolution = bits;
  analogWriteResolution(bits);
  return this;
}

BoardBuilder* BoardBuilder::setAnalogReadResolution(unsigned int bits){
  read_resolution = bits;
  analogReadResolution(bits);
  return this;
}

BoardBuilder* BoardBuilder::setBluetoothTxPort(unsigned int pin){
  board->setBluetoothTxPort(port_factory->createTxPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setBluetoothRxPort(unsigned int pin){
  board->setBluetoothRxPort(port_factory->createRxPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseLeftToePort(unsigned int pin){
  board->setFsrSenseLeftToePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseLeftHeelPort(unsigned int pin){
  board->setFsrSenseLeftHeelPort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseRightToePort(unsigned int pin){
  board->setFsrSenseRightToePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setFsrSenseRightHeelPort(unsigned int pin){
  board->setFsrSenseRightHeelPort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorLeftKneePort(unsigned int pin){
  board->setTorqueSensorLeftKneePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorLeftAnklePort(unsigned int pin){
  board->setTorqueSensorLeftAnklePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorRightKneePort(unsigned int pin){
  board->setTorqueSensorRightKneePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setTorqueSensorRightAnklePort(unsigned int pin){
  board->setTorqueSensorRightAnklePort(port_factory->createAnalogInputPort(pin, read_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorLeftKneePort(unsigned int pin){
  board->setMotorLeftKneePort(port_factory->createPwmOutputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorLeftAnklePort(unsigned int pin){
  board->setMotorLeftAnklePort(port_factory->createPwmOutputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorRightKneePort(unsigned int pin){
  board->setMotorRightKneePort(port_factory->createPwmOutputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setMotorRightAnklePort(unsigned int pin){
  board->setMotorRightAnklePort(port_factory->createPwmOutputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setLedPort(unsigned int pin){
  board->setLedPort(port_factory->createDigitalOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorEnablePort(unsigned int pin){
  board->setMotorEnablePort(port_factory->createDigitalOutputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorLeftKneePort(unsigned int pin){
  board->setMotorErrorLeftKneePort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorLeftAnklePort(unsigned int pin){
  board->setMotorErrorLeftAnklePort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorRightKneePort(unsigned int pin){
  board->setMotorErrorRightKneePort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setMotorErrorRightAnklePort(unsigned int pin){
  board->setMotorErrorRightAnklePort(port_factory->createDigitalInputPort(pin));
  return this;
}

BoardBuilder* BoardBuilder::setPotLeftKneePort(unsigned int pin){
  board->setPotLeftKneePort(port_factory->createAnalogInputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setPotRightKneePort(unsigned int pin){
  board->setPotRightKneePort(port_factory->createAnalogInputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setPotLeftAnklePort(unsigned int pin){
  board->setPotLeftAnklePort(port_factory->createAnalogInputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setPotRightAnklePort(unsigned int pin){
  board->setPotRightAnklePort(port_factory->createAnalogInputPort(pin, write_resolution));
  return this;
}

BoardBuilder* BoardBuilder::setImuSlot0(i2c_pins pins){
  board->setImuSlot0(port_factory->createImuPort(pins));
  return this;
}

BoardBuilder* BoardBuilder::setImuSlot1(i2c_pins pins){
  board->setImuSlot1(port_factory->createImuPort(pins));
  return this;
}

BoardBuilder* BoardBuilder::setImuSlot2(i2c_pins pins){
  board->setImuSlot2(port_factory->createImuPort(pins));
  return this;
}

BoardBuilder* BoardBuilder::setImuAddress0(int address){
  board->setImuAddress0(address);
  return this;
}

BoardBuilder* BoardBuilder::setImuAddress1(int address){
  board->setImuAddress1(address);
  return this;
}
