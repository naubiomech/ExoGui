#include "Arduino.hpp"
#include "FSR.hpp"
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"

FSR::FSR(InputPort* port){
  this->port = port;
  max_force = new Max();
  force_clamp = new Clamp(0,1);
  calibration_peak = 1.0;
  force = 0;
}

FSR::~FSR(){
  delete port;
  delete max_force;
  delete force_clamp;
}

void FSR::measureForce(){
  double force = port->read();

  force = adjustForce(force);
  updateForce(force);
}

void FSR::updateForce(double force){
  max_force->update(force);
  force /= calibration_peak;
  this->force = force_clamp->clamp(force);
}

void FSR::calibrate(){
  calibration_peak = max_force->getMax();
  resetMaxes();
}

void FSR::resetMaxes(){
  max_force->reset();
}

double FSR::getForce(){
  return force;
}

FSRType10::FSRType10(InputPort* port):FSR(port){}

double FSRType10::adjustForce(double force){
  return force;
}

FSRType40::FSRType40(InputPort* port):FSR(port){}

double FSRType40::adjustForce(double force){
  return p[0] * pow(force, 3) + p[1] * pow(force, 2) + p[2] * force + p[3];
}


FSRGroup::FSRGroup(LinkedList<FSR*>* fsrs){
  this->fsr_count = fsrs->size();
  this->fsrs = *fsrs;

  force = 0;
  fsr_percent_thresh = FSR_UPPER_THRESHOLD;
  is_activated  = false;
  activation_threshold = new Threshold(0, FSR_UPPER_THRESHOLD, FSR_LOWER_THRESHOLD);
}

FSRGroup::~FSRGroup(){
  delete activation_threshold;

  ListIterator<FSR*> iter = fsrs.getIterator();

  while(iter.hasNext()){
    delete iter.next();
  }
}

bool FSRGroup::isActivated(){
  return is_activated;
}

void FSRGroup::measureForce(){
  double average = 0;
  for (int i = 0; i < fsr_count; i++){
    fsrs[i]->measureForce();
    average += fsrs[i]->getForce();
  }
  force = average / (double) fsr_count;
  is_activated = activation_threshold->getState(force);
}

void FSRGroup::calibrate(){
  for (int i = 0; i < fsr_count; i++){
    fsrs[i]->calibrate();
  }
}

void FSRGroup::setPercentageThreshold(double percent){
  fsr_percent_thresh = percent;
  activation_threshold->setUpperThreshold(percent);
}

double FSRGroup::getThreshold(){
  return fsr_percent_thresh;
}

void FSRGroup::resetMaxes(){
  for(unsigned int i = 0; i < fsrs.size(); i++){
    fsrs[i]->resetMaxes();
  }
}

double FSRGroup::getPercentage(){
  return force;
}

double FSRGroup::getMaxPercentage(){
  return 1;
}

FSRReport* FSRGroup::generateReport(){
  FSRReport* report = new FSRReport();
  fillLocalReport(report);
  return report;
}

void FSRGroup::fillReport(FSRReport* report){
  fillLocalReport(report);
}

void FSRGroup::fillLocalReport(FSRReport* report){
  report->threshold = getThreshold();
  report->measuredForce = getPercentage();
}

double FSRGroup::getRatio(){
  return 1;
}

double FSRGroup::getDifference(){
  return 0;
}

FsrPair::FsrPair(LinkedList<FSR*>* fsrs):FSRGroup(fsrs){}

double FsrPair::getRatio(){
  return fsrs[0]->getForce()/fsrs[1]->getForce();
}

double FsrPair::getDifference(){
  return fsrs[0]->getForce()-fsrs[1]->getForce();
}

FsrFactory::FsrFactory(int type){
  this->type = type;
}

FSR* FsrFactory::createFSR(InputPort* port){
  FSR* fsr = NULL;
  if (type == 10){
    fsr = new FSRType10(port);
  } else if (type == 40){
    fsr = new FSRType40(port);
  }
  return fsr;
}

FSRGroup* FsrGroupFactory::createFsrGroup(LinkedList<FSR*>* fsrs){
  if (fsrs->size() == 2){
    return new FsrPair(fsrs);
  } else {
    return new FSRGroup(fsrs);
  }
}
