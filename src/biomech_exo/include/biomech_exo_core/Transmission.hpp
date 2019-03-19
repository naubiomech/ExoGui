#ifndef TRANSMISSION_HEADER
#define TRANSMISSION_HEADER
#include "Commands.hpp"
#include "Command_Codes.hpp"
#include "Message.hpp"
#include "Transceiver.hpp"
#include "Report.hpp"


class Transmission{
private:
	unsigned int send_count;
	unsigned int receive_count;
	CommandCode code;

	void getData();
	void sendData();
protected:
	Transceiver* transceiver;
	double* send_data;
	double* receive_data;

	virtual void processData(ExoMessageBuilder* builder, ExoReport* report) = 0;
	void copyToSend(double* from);
	void copyFromReceive(double* to);
public:
	Transmission(Transceiver* transceiver, CommandCode code,
				 unsigned int receive_count, unsigned int send_count);
	virtual ~Transmission();
	void process(ExoMessageBuilder* builder, ExoReport* report);
};

class RequestDataTransmission:public Transmission{
public:
	RequestDataTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class StartTrialTransmission:public Transmission{
public:
	StartTrialTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class EndTrialTransmission:public Transmission{
public:
	EndTrialTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CalibrateTorqueTransmission:public Transmission{
public:
	CalibrateTorqueTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CheckBluetoothTransmission:public Transmission{
public:
	CheckBluetoothTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CleanBluetoothBufferTransmission:public Transmission{
public:
	CleanBluetoothBufferTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetLeftAnkleSetpointTransmission:public Transmission{
public:
	GetLeftAnkleSetpointTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetRightAnkleSetpointTransmission:public Transmission{
public:
	GetRightAnkleSetpointTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetLeftAnkleSetpointTransmission:public Transmission{
public:
	SetLeftAnkleSetpointTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetRightAnkleSetpointTransmission:public Transmission{
public:
	SetRightAnkleSetpointTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CalibrateFsrTransmission:public Transmission{
public:
	CalibrateFsrTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetLeftAnkleFsrThresholdTransmission:public Transmission{
public:
	GetLeftAnkleFsrThresholdTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetRightAnkleFsrThresholdTransmission:public Transmission{
public:
	GetRightAnkleFsrThresholdTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetLeftAnkleKFTransmission:public Transmission{
public:
	GetLeftAnkleKFTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetRightAnkleKFTransmission:public Transmission{
public:
	GetRightAnkleKFTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetLeftAnkleKFTransmission:public Transmission{
public:
	SetLeftAnkleKFTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetRightAnkleKFTransmission:public Transmission{
public:
	SetRightAnkleKFTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};


class GetLeftAnklePidParamsTransmission:public Transmission{
public:
	GetLeftAnklePidParamsTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetRightAnklePidParamsTransmission:public Transmission{
public:
	GetRightAnklePidParamsTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetLeftAnklePidParamsTransmission:public Transmission{
public:
	SetLeftAnklePidParamsTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetRightAnklePidParamsTransmission:public Transmission{
public:
	SetRightAnklePidParamsTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};


class GetSmoothingParamsTransmission:public Transmission{
public:
	GetSmoothingParamsTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetSmoothingParamsTransmission:public Transmission{
public:
	SetSmoothingParamsTransmission(Transceiver* transceiver);
private:
	virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};



class TransmissionFactory{
public:
	Transmission* create(Transceiver* transceiver, CommandCode code);
};
#endif
