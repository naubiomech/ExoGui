#include "ArduinoInterface.hpp"
#include "Arduino.hpp"
#include "Pin.hpp"
#include "ros/console.h"

PinOut* pin_outs[64];
PinIn* pin_ins[64];

void pinMode(int pin,int mode){

	ROS_DEBUG("Creating pin");
	std::string pin_str = "Pin" + std::to_string(pin);

	ros::NodeHandle node_handle;

	if (mode == INPUT){
		pin_ins[pin] = new PinIn(node_handle, pin_str);
	} else if (mode == OUTPUT){
		pin_outs[pin] = new PinOut(node_handle, pin_str);
	}
}

void digitalWrite(int pin, int data){
	pin_outs[pin]->write(data);
}

void analogWrite(int pin, int data){
	pin_outs[pin]->write(data);
}

int digitalRead(int pin){
	return pin_ins[pin]->read();
}

int analogRead(int pin){
	return pin_ins[pin]->read();
}

SoftwareSerial Serial;

SoftwareSerial::SoftwareSerial(){}

SoftwareSerial::SoftwareSerial(int tx, int rx){
	this->rx = rx;
	this->tx = tx;
	pinMode(rx, INPUT);
	pinMode(tx, OUTPUT);
}
bool SoftwareSerial::begin(int){return true;}
void SoftwareSerial::write(char str){
	pin_outs[tx]->write(str);
}
void SoftwareSerial::write(const char* str){
	for (int i = 0; str[i] != 0; i++){
		pin_outs[tx]->write(str[i]);
	}
}

void SoftwareSerial::print(const char* str){
	this->write(str);
}

void SoftwareSerial::print(double val){
	char str[100];
	sprintf(str, "%lf", val);
	this->write(str);
}

void SoftwareSerial::println(){
	pin_outs[tx]->write('\n');
}

void SoftwareSerial::println(const char* str){
	this->print(str);
	this->println();
}

void SoftwareSerial::println(double val){
	this->print(val);
	this->println();
}

int SoftwareSerial::read(){
	if (!this->available()){
		return -1;
	}

	return pin_ins[rx]->read();
}

int SoftwareSerial::available(){return pin_ins[rx]->available();}
