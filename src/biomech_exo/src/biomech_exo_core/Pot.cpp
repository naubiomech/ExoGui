#include "Pot.hpp"

Pot::Pot(InputPort* port){
  this->port = port;
}

Pot::~Pot(){
  delete port;
}

void Pot::measure(){
  angle = port->read() * 290.0;
}

double Pot::getAngle(){
  return angle;
}

PotReport* Pot::generateReport(){
  PotReport* report = new PotReport();
  fillReport(report);
  return report;
}

void Pot::fillReport(PotReport* report){
  report->angle = this->getAngle();
}
