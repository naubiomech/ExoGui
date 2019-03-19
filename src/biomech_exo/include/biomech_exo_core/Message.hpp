#ifndef EXO_MESSAGE_HEADER
#define EXO_MESSAGE_HEADER
#include <cstddef>
#include "Report.hpp"
#include "Linked_List.hpp"
#include "Commands.hpp"

class Exoskeleton;
class Leg;
class Joint;

template <class Context>
class Message{
private:
  LinkedList<Command<Context>*>* pre_commands;
  LinkedList<Command<Context>*>* post_commands;
  void runCommands(Context* context, LinkedList<Command<Context>*>* cmds);
public:
  Message(LinkedList<Command<Context>*>* commands);
  Message(LinkedList<Command<Context>*>* pre_commands, LinkedList<Command<Context>*>* post_commands);
  virtual ~Message();
  void runPreCommands(Context* context);
  void runCommands(Context* context);
  void runPostCommands(Context* context);
};

class JointMessage:public Message<Joint>{
public:
  JointMessage(LinkedList<Command<Joint>*>* commands);
};

class LegMessage:public Message<Leg>{
private:
  LinkedList<JointMessage*>* joint_msgs;
public:
  LegMessage(LinkedList<Command<Leg>*>* pre_commands, LinkedList<Command<Leg>*>* post_commands,
             LinkedList<JointMessage*>* joint_msgs);
  ~LegMessage();
  void messageJoints(LinkedList<Joint*>* joints);
};

class ExoMessage:public Message<Exoskeleton>{
private:
  LegMessage* right_leg_msg;
  LegMessage* left_leg_msg;
public:
  ExoMessage(LinkedList<Command<Exoskeleton>*>* pre_commands, LinkedList<Command<Exoskeleton>*>* post_commands,
             LegMessage* right_leg_msg, LegMessage* left_leg_msg);
  ~ExoMessage();
  void messageRightLeg(Leg* right);
  void messageLeftLeg(Leg* left);
};



template <class T>
class MessageBuilder{
private:
  LinkedList<Command<T>*>  pre_commands;
  LinkedList<Command<T>*>  post_commands;
protected:
  LinkedList<Command<T>*>& getPreCommands();
  LinkedList<Command<T>*>& getPostCommands();
  void clearCommands();
public:
  virtual ~MessageBuilder();
  void addPreCommand(Command<T>* command);
  void addPostCommand(Command<T>* command);
};

template <class T>
class SingleMessageBuilder:private MessageBuilder<T>{
protected:
  LinkedList<Command<T>*>& getCommands();
  void clearCommands();
public:
  void addCommand(Command<T>* command);
};
class JointMessageBuilder;
class LegMessageBuilder;
class ExoMessageBuilder;


class JointMessageBuilder:public SingleMessageBuilder<Joint>{
private:
  LegMessageBuilder* return_context;
public:
  JointMessageBuilder* addCommand(Command<Joint>* command);
  JointMessageBuilder(LegMessageBuilder* return_context);
  LegMessageBuilder* finishJoint();
  JointMessage* build();
};

class LegMessageBuilder:public MessageBuilder<Leg>{
private:
  ExoMessageBuilder* return_context;
  LinkedList<JointMessageBuilder*> joint_builders;
public:
  LegMessageBuilder(ExoMessageBuilder* return_context);
  ~LegMessageBuilder();
  LegMessageBuilder* addPreCommand(Command<Leg>* command);
  LegMessageBuilder* addPostCommand(Command<Leg>* command);
  JointMessageBuilder* beginJointMessage(unsigned int id);
  ExoMessageBuilder* finishLeg();
  LegMessage* build();
};

class ExoMessageBuilder:public MessageBuilder<Exoskeleton>{
private:
  LegMessageBuilder* left_builder;
  LegMessageBuilder* right_builder;
public:
  ExoMessageBuilder();
  ~ExoMessageBuilder();
  ExoMessageBuilder* addPreCommand(Command<Exoskeleton>* command);
  ExoMessageBuilder* addPostCommand(Command<Exoskeleton>* command);
  LegMessageBuilder* beginLeftLegMessage();
  LegMessageBuilder* beginRightLegMessage();
  ExoMessage* build();
};

template<class Context>
Message<Context>::Message(LinkedList<Command<Context>*>* commands){
  pre_commands = commands;
  post_commands = NULL;
}

template<class Context>
Message<Context>::Message(LinkedList<Command<Context>*>* pre_commands, LinkedList<Command<Context>*>* post_commands){
  this->pre_commands = pre_commands;
  this->post_commands = post_commands;
}

template<class Context>
Message<Context>::~Message(){
  pre_commands->deleteItems();

  if (post_commands != NULL){
    post_commands->deleteItems();
  }

  delete pre_commands;
  delete post_commands;
}

template<class Context>
void Message<Context>::runCommands(Context* context, LinkedList<Command<Context>*>* cmds){
  if (cmds == NULL){
    return;
  }

  ListIterator<Command<Context>*> iter = cmds->getIterator();
  while(iter.hasNext()){
    Command<Context>* cmd = iter.next();
    cmd->execute(context);
  }
}

template<class Context>
void Message<Context>::runPreCommands(Context* context){
  runCommands(context, pre_commands);
}

template<class Context>
void Message<Context>::runCommands(Context* context){
  runPreCommands(context);
}

template<class Context>
void Message<Context>::runPostCommands(Context* context){
  runCommands(context, post_commands);
}

template<class T>
MessageBuilder<T>::~MessageBuilder(){
  pre_commands.deleteItems();
  post_commands.deleteItems();
}

template<class T>
void MessageBuilder<T>::addPreCommand(Command<T>* command){
  pre_commands.append(command);
}

template<class T>
void MessageBuilder<T>::addPostCommand(Command<T>* command){
  post_commands.append(command);
}

template<class T>
LinkedList<Command<T>*>& MessageBuilder<T>::getPreCommands(){
  return pre_commands;
}

template<class T>
LinkedList<Command<T>*>& MessageBuilder<T>::getPostCommands(){
  return post_commands;
}

template<class T>
void MessageBuilder<T>::clearCommands(){
  pre_commands.clear();
  post_commands.clear();
}

template<class T>
void SingleMessageBuilder<T>::addCommand(Command<T>* command){
  this->addPreCommand(command);
}

template<class T>
LinkedList<Command<T>*>& SingleMessageBuilder<T>::getCommands(){
  return this->getPreCommands();
}

template<class T>
void SingleMessageBuilder<T>::clearCommands(){
  MessageBuilder<T>::clearCommands();
}
#endif
