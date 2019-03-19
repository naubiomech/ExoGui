#include "States.hpp"
#include "Leg.hpp"

State::State(){
  state_time = new Timer();
  next_state = NULL;
}

State::~State(){
  delete state_time;
}

void State::deleteStateMachine(){
  if (this == NULL || next_state == NULL){
    return;
  }

  State* next = next_state;
  next_state = NULL;
  next->deleteStateMachine();
  delete this;
}

void State::triggerStart(){
  state_time->reset();
}

void State::triggerEnd(){
}

double State::getStateTime(){
  return state_time->lap();
}

void State::setNextState(State* state){
  next_state = state;
}

State* State::changeState(){
  this->triggerEnd();
  next_state->setContext(leg);
  next_state->triggerStart();
  return next_state;
}

void State::setContext(Leg* leg){
  this->leg = leg;
}

void SwingState::run(){
  leg->runAutoKF();
}

StateID SwingState::getStateID(){
  return getStateType();
}

StateType SwingState::getStateType(){
  return SWING;
}

void LateStanceState::triggerStart(){
  State::triggerStart();
  leg->resetFSRMaxes();
  leg->incrementStepCount();
  leg->setZeroIfNecessary();
}

void LateStanceState::run(){

}

void LateStanceState::triggerEnd(){
  State::triggerEnd();
  leg->adjustShapingForTime(getStateTime());
}

StateID LateStanceState::getStateID(){
  return getStateType();
}

StateType LateStanceState::getStateType(){
  return LATE_STANCE;
}

StateBuilder* StateBuilder::addState(StateType state_type){
  states.append(state_type);
  return this;
}

State* StateBuilder::build(){
  State* first = makeState(states[0]);
  State* previous = first;
  for(unsigned int i = 1; i < states.size(); i++){
    State* current = makeState(states[i]);
    previous->setNextState(current);
    previous = current;
  }
  previous->setNextState(first);
  return first;
}

State* StateBuilder::makeState(StateType state_type){
  switch(state_type){
  case SWING:
    return new SwingState();
  case LATE_STANCE:
    return new LateStanceState();
  default:
    return NULL;
  }
}
