#ifndef PORT_HEADER
#define PORT_HEADER

#include "Arduino.hpp"

class Port{
private:
  unsigned int pin;
protected:
  unsigned int getPin();
public:
  Port(unsigned int pin);
  virtual ~Port();
};

class ImuPort:public Port{
private:
  i2c_pins imu_pins;
  i2c_bus imu_bus;
public:
  ImuPort(i2c_pins imu_pins);
  i2c_pins getPins();
  i2c_bus getBus();
  virtual ~ImuPort();
};

class InputPort:public Port{
public:
  InputPort(unsigned int pin);
  virtual double read() = 0;
  virtual ~InputPort();
};

class AnalogInputPort: public InputPort{
private:
  int resolution;
public:
  double read();
  AnalogInputPort(unsigned int pin, unsigned int resolution_bits);
  virtual ~AnalogInputPort();
};

class DigitalInputPort: public InputPort{
public:
  double read();
  DigitalInputPort(unsigned int pin);
  virtual ~DigitalInputPort();
};

class RxPort:public InputPort{
public:
  double read();
  unsigned int getPin();
  RxPort(unsigned int pin);
  virtual ~RxPort();
};

class OutputPort: public Port{
public:
  virtual void write(double value) = 0;
  OutputPort(unsigned int pin);
  virtual ~OutputPort();
};

class AnalogOutputPort: public OutputPort{
private:
  double resolution;
public:
  void write(double value);
  AnalogOutputPort(unsigned int pin, unsigned int bit_resolution);
  virtual ~AnalogOutputPort();
};

class DigitalOutputPort: public OutputPort{
public:
  void write(double value);
  DigitalOutputPort(unsigned int pin);
  virtual ~DigitalOutputPort();
};

class PwmOutputPort: public AnalogOutputPort{
public:
  void write(double value);
  PwmOutputPort(unsigned int pin, unsigned int resolution_bits);
  virtual ~PwmOutputPort();
};

class TxPort:public OutputPort{
public:
  void write(double value);
  unsigned int getPin();
  TxPort(unsigned int pin);
  virtual ~TxPort();
};

class PortFactory{
public:
  virtual ~PortFactory();
  virtual RxPort* createRxPort(unsigned int pin) = 0;
  virtual TxPort* createTxPort(unsigned int pin) = 0;
  virtual InputPort* createDigitalInputPort(unsigned int pin) = 0;
  virtual InputPort* createAnalogInputPort(unsigned int pin, unsigned int resolution_bits) = 0;
  virtual OutputPort* createDigitalOutputPort(unsigned int pin) = 0;
  virtual OutputPort* createAnalogOutputPort(unsigned int pin, unsigned int resolution_bits) = 0;
  virtual OutputPort* createPwmOutputPort(unsigned int pin, unsigned int resolution_bits) = 0;
  virtual ImuPort* createImuPort(i2c_pins pins) = 0;
};

class ArduinoPortFactory:public PortFactory{
  RxPort* createRxPort(unsigned int pin);
  TxPort* createTxPort(unsigned int pin);
  InputPort* createDigitalInputPort(unsigned int pin);
  InputPort* createAnalogInputPort(unsigned int pin, unsigned int resolution_bits);
  OutputPort* createDigitalOutputPort(unsigned int pin);
  OutputPort* createAnalogOutputPort(unsigned int pin, unsigned int resolution_bits);
  OutputPort* createPwmOutputPort(unsigned int pin, unsigned int resolution_bits);
  ImuPort* createImuPort(i2c_pins pins);
};

#endif
