#include "ros/ros.h"
#include "Pin.hpp"
#include "std_msgs/Int16.h"

PinIn::PinIn(ros::NodeHandle& node, const std::string& portName){
	sub = node.subscribe(portName, 100, &PinIn::callback, this);
}

void PinIn::callback(const std_msgs::Int16::ConstPtr& update_msg){
	this->received = update_msg->data;
}

int PinIn::read(){
	return received;
}

PinOut::PinOut(ros::NodeHandle& node, const std::string& portName){
	pub = node.advertise<std_msgs::Int16>(portName, 100);
}

void PinOut::write(int val){
	std_msgs::Int16 msg;
	msg.data = val;
	pub.publish(msg);
}
