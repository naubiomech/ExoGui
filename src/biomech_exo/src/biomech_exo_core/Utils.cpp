#include "Arduino.hpp"
#include "Utils.hpp"
#include <float.h>

RunningAverage::RunningAverage(){
  avg = 0;
  count = 0;
}

double RunningAverage::update(double value){
  avg += value;
  count++;
  return getAverage();
}

double RunningAverage::getAverage(){
  if (count == 0){
    return 0;
  }
  return avg / count;
}

void RunningAverage::reset(){
  avg = 0;
  count = 0;
}

MovingAverage::MovingAverage(int size){
  previous_values = new double[size];
  this->size = size;
  this->reset();
}

MovingAverage::~MovingAverage(){
  delete[] previous_values;
}

void MovingAverage::reset(){
  for(int i = 0; i < size;i++){
    previous_values[i] = 0;
  }
  this->total = 0;
  this->average = 0;
  this->index = 0;
}

double MovingAverage::update(double value){
  total -= previous_values[index];
  previous_values[index] = value;
  index = (index + 1) % size;
  total += value;
  average = total / size;
  return average;
}

double MovingAverage::getAverage(){
  return this->average;
}

Clamp::Clamp(double lower, double upper){
  this->upper = upper;
  this->lower = lower;
}

double Clamp::clamp(double val){
  return min(max(val, lower),upper);
}

void updateMax(double* max_value, double value){
  if (*max_value > value) {
    *max_value = value;
  }
}

Threshold::Threshold(bool starting_state, double upper_threshold_value, double lower_threshold_value){
  this->upper_threshold = upper_threshold_value;
  this->lower_threshold = lower_threshold_value;
  this->state = starting_state;
}

bool Threshold::getState(double value){
  if (state == 0 && value > upper_threshold){
    state = 1;
  } else if (state == 1 && value < lower_threshold){
    state = 0;
  }
  return state;
}

void Threshold::setUpperThreshold(double threshold){
  upper_threshold = threshold;
}

void Threshold::setLowerThreshold(double threshold){
  lower_threshold = threshold;
}

void Range::update(double value){
  maximum->update(value);
  minimum->update(value);
  avg = (maximum->getMax() + minimum->getMin())/2;
  threshold->setUpperThreshold(avg);
  threshold->setLowerThreshold(avg);
  bool state = threshold->getState(value);
  bool state_change = trigger->update(state);
  if(state_change && state == false){
    avgMax->update(maximum->getMax());
    maximum->reset();
  } else if (state_change && state == true){
    avgMin->update(minimum->getMin());
    minimum->reset();
  }
}

Timer::Timer(){
  start_time = millis();
  pause_time = 0;
}

double Timer::lapSec(){
  return this->lap() / 1000.0;
}

double Timer::lap(){
  return millis() - start_time;
}

void Timer::reset(){
  start_time = millis();
}

void Timer::pause(){
  pause_time = millis();
}

void Timer::resume(){
  start_time -= (millis() - pause_time);
  pause_time = 0;
}

Max::Max(){
  this->reset();
}

double Max::getMax(){
  return this->maxVal;
}

void Max::update(double value){
  maxVal = max(value, maxVal);
}

void Max::reset(){
  maxVal = FLT_MIN;
}

Min::Min(){
  this->reset();
}

double Min::getMin(){
  return this->minVal;
}

void Min::update(double value){
  minVal = min(value, minVal);
}

void Min::reset(){
  minVal = FLT_MAX;
}

ChangeTrigger::ChangeTrigger(bool start_state){
  state = start_state;
}

bool ChangeTrigger::update(bool state){
  if (this->state != state){
    this->state = state;
    return true;
  }
  return false;
}
