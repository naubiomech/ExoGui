#include "Message.hpp"
#include "Leg.hpp"
#include "Joint.hpp"

JointMessage::JointMessage(LinkedList<Command<Joint>*>* commands):Message<Joint>(commands){}

LegMessage::LegMessage(LinkedList<Command<Leg>*>* pre_commands,
                       LinkedList<Command<Leg>*>* post_commands,
                       LinkedList<JointMessage*>* joint_msgs):Message<Leg>(pre_commands, post_commands){
                         this->joint_msgs = joint_msgs;
}

LegMessage::~LegMessage(){
  joint_msgs->deleteItems();
  delete joint_msgs;
}

void LegMessage::messageJoints(LinkedList<Joint*>* joints){
  ListIterator<Joint*> joint_iter = joints->getIterator();
  ListIterator<JointMessage*> msg_iter = joint_msgs->getIterator();
  while(joint_iter.hasNext() && msg_iter.hasNext()){
    joint_iter.next()->processMessage(msg_iter.next());
  }
}

ExoMessage::ExoMessage(LinkedList<Command<Exoskeleton>*>* pre_commands,
                       LinkedList<Command<Exoskeleton>*>* post_commands,
                       LegMessage* right_leg_msg,
                       LegMessage* left_leg_msg):Message<Exoskeleton>(pre_commands, post_commands){
                         this->right_leg_msg = right_leg_msg;
                         this->left_leg_msg = left_leg_msg;
}

ExoMessage::~ExoMessage(){
  delete left_leg_msg;
  delete right_leg_msg;
}

void ExoMessage::messageLeftLeg(Leg* left){
  left->processMessage(left_leg_msg);
}

void ExoMessage::messageRightLeg(Leg* right){
  right->processMessage(right_leg_msg);
}



JointMessageBuilder::JointMessageBuilder(LegMessageBuilder* return_context){
  this->return_context = return_context;
}

JointMessageBuilder* JointMessageBuilder::addCommand(Command<Joint>* command){
  SingleMessageBuilder<Joint>::addCommand(command);
  return this;
}

LegMessageBuilder* JointMessageBuilder::finishJoint(){
  return return_context;
}

JointMessage* JointMessageBuilder::build(){
  if (this == NULL){
    return NULL;
  }

  JointMessage* joint_msg = new JointMessage(getCommands().copy());
  clearCommands();
  return joint_msg;
}

LegMessageBuilder::LegMessageBuilder(ExoMessageBuilder* return_context){
  this->return_context = return_context;
}

LegMessageBuilder::~LegMessageBuilder(){
  joint_builders.deleteItems();
}

LegMessageBuilder* LegMessageBuilder::addPreCommand(Command<Leg>* command){
  MessageBuilder<Leg>::addPreCommand(command);
  return this;
}

LegMessageBuilder* LegMessageBuilder::addPostCommand(Command<Leg>* command){
  MessageBuilder<Leg>::addPostCommand(command);
  return this;
}

JointMessageBuilder* LegMessageBuilder::beginJointMessage(unsigned int id){
  while (id >= joint_builders.size()){
    joint_builders.append(new JointMessageBuilder(this));
  }
  return joint_builders[id];
}

ExoMessageBuilder* LegMessageBuilder::finishLeg(){
  return return_context;
}

LegMessage* LegMessageBuilder::build(){
  if (this == NULL){
    return NULL;
  }

  LinkedList<JointMessage*>* joint_msgs = new LinkedList<JointMessage*>();
  ListIterator<JointMessageBuilder*> joint_builder_iter = joint_builders.getIterator();
  while(joint_builder_iter.hasNext()){
    joint_msgs->append(joint_builder_iter.next()->build());
  }
  LegMessage* leg_msg = new LegMessage(getPreCommands().copy(), getPostCommands().copy(), joint_msgs);
  clearCommands();
  return leg_msg;
}

ExoMessageBuilder::ExoMessageBuilder(){
  left_builder = NULL;
  right_builder = NULL;
}

ExoMessageBuilder::~ExoMessageBuilder(){
  delete left_builder;
  delete right_builder;
}

ExoMessageBuilder* ExoMessageBuilder::addPreCommand(Command<Exoskeleton>* command){
  MessageBuilder<Exoskeleton>::addPreCommand(command);
  return this;
}

ExoMessageBuilder* ExoMessageBuilder::addPostCommand(Command<Exoskeleton>* command){
  MessageBuilder<Exoskeleton>::addPostCommand(command);
  return this;
}

LegMessageBuilder* ExoMessageBuilder::beginLeftLegMessage(){
  if(left_builder == NULL){
    left_builder = new LegMessageBuilder(this);
  }
  return left_builder;
}

LegMessageBuilder* ExoMessageBuilder::beginRightLegMessage(){
  if(right_builder == NULL){
    right_builder = new LegMessageBuilder(this);
  }
  return right_builder;
}

ExoMessage* ExoMessageBuilder::build(){
  if (this == NULL){
    return NULL;
  }

  LegMessage* left_msg = left_builder->build();
  LegMessage* right_msg = right_builder->build();

  ExoMessage* exo_msg = new ExoMessage(getPreCommands().copy(), getPostCommands().copy(), right_msg, left_msg);
  this->clearCommands();
  return exo_msg;
}
