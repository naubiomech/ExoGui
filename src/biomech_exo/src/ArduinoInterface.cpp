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
