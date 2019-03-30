#include "ros/ros.h"
#include "Pin.hpp"
#include "std_msgs/Int16.h"
#include <queue>


PinIn::PinIn(ros::NodeHandle& node, const std::string& portName){
	sub = node.subscribe(portName, 10, &PinIn::callback, this);
}

void PinIn::callback(const std_msgs::Int16::ConstPtr& update_msg){
	if (this->queue.size() < 100){
		this->queue.push(update_msg->data);
	}
}

int PinIn::read(){
	if (this->queue.size() > 0){
		this->received = this->queue.front();
		this->queue.pop();
	}
	return received;
}

unsigned int PinIn::available(){
	return this->queue.size();
}

PinOut::PinOut(ros::NodeHandle& node, const std::string& portName){
	pub = node.advertise<std_msgs::Int16>(portName, 100);
}

void PinOut::write(int val){
	std_msgs::Int16 msg;
	msg.data = val;
	pub.publish(msg);
}
