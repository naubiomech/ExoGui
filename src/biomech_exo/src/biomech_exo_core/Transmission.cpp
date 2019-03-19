#include "Transmission.hpp"
#include "Transceiver.hpp"
#include <string.h>

Transmission::Transmission(Transceiver* transceiver, CommandCode code,
                           unsigned int receive_count, unsigned int send_count){
  this->code = code;
  this->transceiver = transceiver;
  this->send_count = send_count;
  this->receive_count = receive_count;

  if (send_count > 0){
    send_data = new double[send_count];
  } else {
    send_data = NULL;
  }

  if (receive_count > 0){
    receive_data = new double[receive_count];
  } else {
    receive_data = NULL;
  }
}

Transmission::~Transmission(){
  delete[] send_data;
  delete[] receive_data;
}


void Transmission::process(ExoMessageBuilder* builder, ExoReport* report){
  getData();
  processData(builder, report);
  sendData();
}

void Transmission::getData(){
  if (receive_count > 0){
    transceiver->receiveData(receive_data, receive_count);
    transceiver->receiveFooter();
  }
}

void Transmission::sendData(){
  if (send_count > 0){
    transceiver->sendHeader();
    transceiver->sendCommand(code);
    transceiver->sendData(send_data, send_count);
    transceiver->sendFooter();
  }
}

void Transmission::copyToSend(double* from){
  memcpy(send_data, from, sizeof(double) * send_count);
}

void Transmission::copyFromReceive(double* to){
  memcpy(to, receive_data, sizeof(double) * receive_count);
}

RequestDataTransmission::RequestDataTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_REQUEST_DATA, 0, 14){}
void RequestDataTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->right_leg->joint_reports[0]->torque_sensor_report->measuredTorque;
  send_data[1] = report->right_leg->state;
  send_data[2] = report->right_leg->joint_reports[0]->pid_setpoint;
  send_data[3] = report->right_leg->sensor_reports->fsr_reports[0]->threshold;
  send_data[4] = report->right_leg->sensor_reports->fsr_reports[0]->measuredForce;

  send_data[5] = report->left_leg->joint_reports[0]->torque_sensor_report->measuredTorque;
  send_data[6] = report->left_leg->state;
  send_data[7] = report->left_leg->joint_reports[0]->pid_setpoint;
  send_data[8] = report->left_leg->sensor_reports->fsr_reports[0]->threshold;
  send_data[9] = report->left_leg->sensor_reports->fsr_reports[0]->measuredForce;

  send_data[10] = 0;
  send_data[11] = 0;
  send_data[12] = 0;
  send_data[13] = 0;
}

StartTrialTransmission::StartTrialTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_START_TRIAL, 0, 0){}
void StartTrialTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new StartTrialCommand());
}

EndTrialTransmission::EndTrialTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_END_TRIAL, 0, 0){}
void EndTrialTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new EndTrialCommand());
}

CalibrateTorqueTransmission::CalibrateTorqueTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CALIBRATE_TORQUE, 0, 0){}
void CalibrateTorqueTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new CalibrateAllTorquesCommand());
}

CheckBluetoothTransmission::CheckBluetoothTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CHECK_BLUETOOTH, 0, 3){}
void CheckBluetoothTransmission::processData(ExoMessageBuilder*, ExoReport*){
  send_data[0] = 0;
  send_data[1] = 1;
  send_data[2] = 2;
}

CleanBluetoothBufferTransmission::CleanBluetoothBufferTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CLEAN_BLUETOOTH_BUFFER, 0, 0){}
void CleanBluetoothBufferTransmission::processData(ExoMessageBuilder*, ExoReport*){
  transceiver->clear();
}

GetLeftAnkleSetpointTransmission::GetLeftAnkleSetpointTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_LEFT_ANKLE_SETPOINT, 0, 1){}
void GetLeftAnkleSetpointTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->left_leg->joint_reports[0]->pid_setpoint;
}

GetRightAnkleSetpointTransmission::GetRightAnkleSetpointTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_RIGHT_ANKLE_SETPOINT, 0, 1){}
void GetRightAnkleSetpointTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->right_leg->joint_reports[0]->pid_setpoint;
}

SetLeftAnkleSetpointTransmission::SetLeftAnkleSetpointTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_LEFT_ANKLE_SETPOINT, 2, 0){}
void SetLeftAnkleSetpointTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginLeftLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointSetpointCommand(LATE_STANCE, receive_data[0]))->
    addCommand(new SetJointSetpointCommand(SWING, receive_data[1]));
}

SetRightAnkleSetpointTransmission::SetRightAnkleSetpointTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_RIGHT_ANKLE_SETPOINT, 2, 0){}
void SetRightAnkleSetpointTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginRightLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointSetpointCommand(LATE_STANCE, receive_data[0]))->
    addCommand(new SetJointSetpointCommand(SWING, receive_data[1]));
}

CalibrateFsrTransmission::CalibrateFsrTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CALIBRATE_FSR, 0, 0){}
void CalibrateFsrTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new CalibrateAllFsrsCommand());
}

GetLeftAnkleFsrThresholdTransmission::GetLeftAnkleFsrThresholdTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_LEFT_ANKLE_FSR_THRESHOLD, 0, 1){}
void GetLeftAnkleFsrThresholdTransmission::processData(ExoMessageBuilder*, ExoReport*){
  send_data[0] = 1;
}

GetRightAnkleFsrThresholdTransmission::GetRightAnkleFsrThresholdTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_RIGHT_ANKLE_FSR_THRESHOLD, 0, 1){}
void GetRightAnkleFsrThresholdTransmission::processData(ExoMessageBuilder*, ExoReport*){
  send_data[0] = 1;
}

GetRightAnkleKFTransmission::GetRightAnkleKFTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_RIGHT_ANKLE_KF, 0, 1){}
void GetRightAnkleKFTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->right_leg->joint_reports[0]->pid_kf;
}

GetLeftAnkleKFTransmission::GetLeftAnkleKFTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_LEFT_ANKLE_KF, 0, 1){}
void GetLeftAnkleKFTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->left_leg->joint_reports[0]->pid_kf;
}

SetRightAnkleKFTransmission::SetRightAnkleKFTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_RIGHT_ANKLE_KF, 1, 0){}
void SetRightAnkleKFTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginRightLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointKfCommand(receive_data[0]));
}

SetLeftAnkleKFTransmission::SetLeftAnkleKFTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_LEFT_ANKLE_KF, 1, 0){}
void SetLeftAnkleKFTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginLeftLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointKfCommand(receive_data[0]));
}

GetRightAnklePidParamsTransmission::GetRightAnklePidParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_RIGHT_ANKLE_PID_PARAMS, 0, 3){}
void GetRightAnklePidParamsTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  copyToSend(report->right_leg->joint_reports[0]->pid_params);
}

GetLeftAnklePidParamsTransmission::GetLeftAnklePidParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_LEFT_ANKLE_PID_PARAMS, 0, 3){}
void GetLeftAnklePidParamsTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  copyToSend(report->left_leg->joint_reports[0]->pid_params);
}

SetRightAnklePidParamsTransmission::SetRightAnklePidParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_RIGHT_ANKLE_PID_PARAMS, 3, 0){}
void SetRightAnklePidParamsTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginRightLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointPidCommand(receive_data[0], receive_data[1], receive_data[2]));
}

SetLeftAnklePidParamsTransmission::SetLeftAnklePidParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_LEFT_ANKLE_PID_PARAMS, 3, 0){}
void SetLeftAnklePidParamsTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginLeftLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointPidCommand(receive_data[0], receive_data[1], receive_data[2]));
}

GetSmoothingParamsTransmission::GetSmoothingParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_SMOOTHING_PARAMS, 0, 3){}
void GetSmoothingParamsTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  copyToSend(report->right_leg->joint_reports[0]->smoothing);
}

SetSmoothingParamsTransmission::SetSmoothingParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_SMOOTHING_PARAMS, 3, 0){}
void SetSmoothingParamsTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginRightLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointSmoothingParamCommand(SWING, receive_data[0]))->
    addCommand(new SetJointSmoothingParamCommand(LATE_STANCE, receive_data[2]))->
    finishJoint()->
    finishLeg()->
    beginLeftLegMessage()->
    beginJointMessage(0)->
    addCommand(new SetJointSmoothingParamCommand(SWING, receive_data[0]))->
    addCommand(new SetJointSmoothingParamCommand(LATE_STANCE, receive_data[2]));
}


Transmission* TransmissionFactory::create(Transceiver* trans, CommandCode code){
  switch (code) {
  case COMM_CODE_REQUEST_DATA:
    return new RequestDataTransmission(trans);
  case COMM_CODE_START_TRIAL:
    return new StartTrialTransmission(trans);
  case COMM_CODE_END_TRIAL:
    return new EndTrialTransmission(trans);
  case COMM_CODE_CALIBRATE_TORQUE:
    return new CalibrateTorqueTransmission(trans);
  case COMM_CODE_CHECK_BLUETOOTH:
    return new CheckBluetoothTransmission(trans);
  case COMM_CODE_CLEAN_BLUETOOTH_BUFFER:
    return new CleanBluetoothBufferTransmission(trans);
  case COMM_CODE_GET_LEFT_ANKLE_SETPOINT:
    return new GetLeftAnkleSetpointTransmission(trans);
  case COMM_CODE_GET_RIGHT_ANKLE_SETPOINT:
    return new GetRightAnkleSetpointTransmission(trans);
  case COMM_CODE_SET_LEFT_ANKLE_SETPOINT:
    return new SetLeftAnkleSetpointTransmission(trans);
  case COMM_CODE_SET_RIGHT_ANKLE_SETPOINT:
    return new SetRightAnkleSetpointTransmission(trans);
  case COMM_CODE_CALIBRATE_FSR:
    return new CalibrateFsrTransmission(trans);
  case COMM_CODE_GET_LEFT_ANKLE_FSR_THRESHOLD:
    return new GetLeftAnkleFsrThresholdTransmission(trans);
  case COMM_CODE_GET_RIGHT_ANKLE_FSR_THRESHOLD:
    return new GetRightAnkleFsrThresholdTransmission(trans);
  case COMM_CODE_GET_LEFT_ANKLE_KF:
    return new GetLeftAnkleKFTransmission(trans);
  case COMM_CODE_GET_RIGHT_ANKLE_KF:
    return new GetRightAnkleKFTransmission(trans);
  case COMM_CODE_SET_LEFT_ANKLE_KF:
    return new SetLeftAnkleKFTransmission(trans);
  case COMM_CODE_SET_RIGHT_ANKLE_KF:
    return new SetRightAnkleKFTransmission(trans);
  case COMM_CODE_GET_LEFT_ANKLE_PID_PARAMS:
    return new GetLeftAnklePidParamsTransmission(trans);
  case COMM_CODE_GET_RIGHT_ANKLE_PID_PARAMS:
    return new GetRightAnklePidParamsTransmission(trans);
  case COMM_CODE_SET_LEFT_ANKLE_PID_PARAMS:
    return new SetLeftAnklePidParamsTransmission(trans);
  case COMM_CODE_SET_RIGHT_ANKLE_PID_PARAMS:
    return new SetRightAnklePidParamsTransmission(trans);
  case COMM_CODE_GET_SMOOTHING_PARAMS:
    return new GetSmoothingParamsTransmission(trans);
  case COMM_CODE_SET_SMOOTHING_PARAMS:
    return new SetSmoothingParamsTransmission(trans);
  default:
    Serial.print("Command code not implemented: ");
    Serial.println(code);
  }
  return NULL;
}
