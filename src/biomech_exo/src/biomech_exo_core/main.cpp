#ifndef ARDUINO
#include "Arduino.hpp"
#include "ExoBuilder.hpp"
#include "BoardBuilder.hpp"
#include "Port.hpp"
#include "Linked_List.hpp"
#include "FSR.hpp"
#include "Message.hpp"

Exoskeleton* setupSystem(){
  Serial.begin(115200);
  delay(500);
  Board* board = QuadBoardDirector().build();
  Serial.println("Beginning exo building..");
  Exoskeleton* exo = QuadExoDirector().build(board);
  Serial.println("Finished exo building");
  delete board;
  return exo;
}

void testExo(){
  Serial.setReadString("k");
  Exoskeleton* exo = setupSystem();
  ExoMessageBuilder builder;
  builder.addPreCommand(new StartTrialCommand());
  ExoMessage* msg = builder.build();
  exo->processMessage(msg);
  delete msg;
  exo->receiveMessages();
  exo->run();
  exo->sendReport();
  delete exo;
}

int main(){
  testExo();
}
#endif
