#ifndef STATE_MACHINE_HEADER
#define STATE_MACHINE_HEADER
#include "Utils.hpp"
#include "Linked_List.hpp"

enum StateType {SWING = 1, LATE_STANCE = 3};

typedef StateType StateID;

class Leg;
class State{
private:
  Timer* state_time;
  State* next_state;
protected:
  Leg* leg;
public:
  State();
  virtual ~State();
  void deleteStateMachine();
  double getStateTime();
  State* changeState();
  void setNextState(State* next_phase);
  void setContext(Leg* leg);
  virtual void triggerStart();
  virtual void triggerEnd();
  virtual void run()=0;
  virtual StateType getStateType() = 0;
  virtual StateID getStateID() = 0;
};

class SwingState : public State{
public:
  void run();
  virtual StateID getStateID();
  virtual StateType getStateType();
};

class LateStanceState : public State{
public:
  void triggerStart();
  void run();
  void triggerEnd();
  virtual StateID getStateID();
  virtual StateType getStateType();
};

class StateBuilder{
private:
  LinkedList<StateType> states;
  State* makeState(StateType state_type);
public:
  StateBuilder* addState(StateType state_type);
  State* build();
};

#endif
