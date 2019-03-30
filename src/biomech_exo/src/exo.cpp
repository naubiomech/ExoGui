#include "ros/ros.h"
#include "BoardBuilder.hpp"
#include "Board.hpp"
#include "Exoskeleton.hpp"
#include "ExoBuilder.hpp"
#include "Linked_List.hpp"
#include "Utils.hpp"

Exoskeleton* exo;

ros::NodeHandle* node_handle;
Exoskeleton* setupSystem(){
	Board* board = QuadBoardDirector().build();
	Exoskeleton* exo = QuadExoDirector().build(board);
	delete board;
	return exo;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ArduinoInterface");
	ros::NodeHandle n;
	node_handle = &n;
	ros::Rate loop_rate(2000);
	Serial = SoftwareSerial(100,101);

	exo = setupSystem();

	while(ros::ok()){
		exo->run();
		exo->sendReport();
		exo->receiveMessages();

		loop_rate.sleep();
	}
}

