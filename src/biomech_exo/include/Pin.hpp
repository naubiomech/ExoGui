#ifndef PIN_HEADER
#define PIN_HEADER
#include "ros/ros.h"
#include <string.h>
#include "std_msgs/Int16.h"

class PinIn{
private:
	int received;

	ros::Subscriber sub;
public:
	PinIn(ros::NodeHandle& node, const std::string& portName);
	int read();
	void callback(const std_msgs::Int16::ConstPtr& data);
};

class PinOut{
private:
	ros::Publisher pub;
public:
	PinOut(ros::NodeHandle& node, const std::string& portName);
	void write(int);
};
#endif
