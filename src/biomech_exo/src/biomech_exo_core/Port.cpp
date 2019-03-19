#include "Arduino.hpp"
#include "Port.hpp"

int calculateResolution(unsigned int bits){
  int res = 1 << bits;
  return res;
}

Port::Port(unsigned int pin){
  this->pin = pin;
}
Port::~Port(){}

unsigned int Port::getPin(){
  return pin;
}

ImuPort::ImuPort(i2c_pins pins):Port(-1){
  imu_pins = pins;
  switch(pins){
  case I2C_PINS_7_8:
    imu_bus = WIRE_BUS;
    break;
  case I2C_PINS_3_4:
    imu_bus = WIRE1_BUS;
    break;
  default:
    imu_bus = WIRE_BUS;
    break;
  }
}

ImuPort::~ImuPort(){

}

i2c_pins ImuPort::getPins(){
  return imu_pins;
}

i2c_bus ImuPort::getBus(){
  return imu_bus;
}

InputPort::InputPort(unsigned int pin):Port(pin){}
InputPort::~InputPort(){}

AnalogInputPort::AnalogInputPort(unsigned int pin, unsigned int resolution_bits):InputPort(pin){
  resolution = calculateResolution(resolution_bits);
  pinMode(pin, INPUT);
}

AnalogInputPort::~AnalogInputPort(){}

double AnalogInputPort::read(){
  double readValue = analogRead(getPin());
  return readValue/resolution;
}

DigitalInputPort::DigitalInputPort(unsigned int pin):InputPort(pin){
  pinMode(pin, INPUT);
}
DigitalInputPort::~DigitalInputPort(){}

double DigitalInputPort::read(){
  return digitalRead(getPin());
}

RxPort::RxPort(unsigned int pin):InputPort(pin){}
RxPort::~RxPort(){}

double RxPort::read(){
  return 0;
}

unsigned int RxPort::getPin(){
  return InputPort::getPin();
}

OutputPort::OutputPort(unsigned int pin):Port(pin){}
OutputPort::~OutputPort(){}

AnalogOutputPort::AnalogOutputPort(unsigned int pin, unsigned int resolution_bits):OutputPort(pin){
  resolution = calculateResolution(resolution_bits);
  pinMode(pin, OUTPUT);
}
AnalogOutputPort::~AnalogOutputPort(){}

void AnalogOutputPort::write(double value){
  int output = (int) (value * resolution);
  analogWrite(getPin(), output);
}

DigitalOutputPort::DigitalOutputPort(unsigned int pin):OutputPort(pin){
  pinMode(pin, OUTPUT);
}

DigitalOutputPort::~DigitalOutputPort(){}

void DigitalOutputPort::write(double value){
  int digitalVal = value;
  digitalWrite(getPin(), digitalVal);
}

PwmOutputPort::PwmOutputPort(unsigned int pin, unsigned int resolution_bits):AnalogOutputPort(pin, resolution_bits){}
PwmOutputPort::~PwmOutputPort(){}

void PwmOutputPort::write(double value){
  AnalogOutputPort::write(value * 0.8 + 0.1);
}

TxPort::TxPort(unsigned int pin):OutputPort(pin){}
TxPort::~TxPort(){}

void TxPort::write(double){}

unsigned int TxPort::getPin(){
  return OutputPort::getPin();
}

PortFactory::~PortFactory(){}

RxPort* ArduinoPortFactory::createRxPort(unsigned int pin){
  return new RxPort(pin);
}

TxPort* ArduinoPortFactory::createTxPort(unsigned int pin){
  return new TxPort(pin);
}

InputPort* ArduinoPortFactory::createDigitalInputPort(unsigned int pin){
  return new DigitalInputPort(pin);
}

InputPort* ArduinoPortFactory::createAnalogInputPort(unsigned int pin, unsigned int resolution_bits){
  return new AnalogInputPort(pin, resolution_bits);
}

OutputPort* ArduinoPortFactory::createDigitalOutputPort(unsigned int pin){
  return new DigitalOutputPort(pin);
}

OutputPort* ArduinoPortFactory::createAnalogOutputPort(unsigned int pin, unsigned int resolution_bits){
  return new AnalogOutputPort(pin, resolution_bits);
}

OutputPort* ArduinoPortFactory::createPwmOutputPort(unsigned int pin, unsigned int resolution_bits){
  return new PwmOutputPort(pin, resolution_bits);
}

ImuPort* ArduinoPortFactory::createImuPort(i2c_pins pins){
  return new ImuPort(pins);
}
