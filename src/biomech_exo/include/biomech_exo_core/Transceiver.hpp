#ifndef TRANSCEIVER_HEADER
#define TRANSCEIVER_HEADER
#include "Arduino.hpp"
#include "Port.hpp"
#include "Command_Codes.hpp"
#include "Report.hpp"
#include "Message.hpp"

class Transceiver{
public:
  virtual ~Transceiver();

  virtual bool dataAvailable() = 0;
  virtual bool noDataAvailable();
  virtual void clear() = 0;

  virtual void sendHeader() = 0;
  virtual void sendData(double* data, int bytes_to_send) = 0;
  virtual void sendCommand(CommandCode code) = 0;
  virtual void sendFooter() = 0;

  virtual bool  receiveHeader() = 0;
  virtual CommandCode receiveCommand() = 0;
  virtual void receiveData(double* data_output, int bytes_expected) = 0;
  virtual bool  receiveFooter() = 0;
};

class MatlabTransceiver:public Transceiver{
private:
  SoftwareSerial* serial;
public:
  MatlabTransceiver(TxPort* tx, RxPort* rx);
  ~MatlabTransceiver();

  virtual bool dataAvailable();
  virtual void clear();

  virtual void sendHeader();
  virtual void sendData(double* data, int bytes_to_send);
  virtual void sendCommand(CommandCode code);
  virtual void sendFooter();

  virtual bool  receiveHeader();
  virtual void receiveData(double* data_output, int bytes_expected);
  virtual CommandCode receiveCommand();
  virtual bool  receiveFooter();
};

#endif
