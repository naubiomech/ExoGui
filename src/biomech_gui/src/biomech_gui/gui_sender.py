#!/usr/bin/env python
import os
import time

import rospy

from biomech_comms.msg import ExoCommand
from biomech_comms.comm_codes import CommandCode

class ExoGuiSender():
    def __init__(self, handler, pub_topic, data):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        publisher = rospy.Publisher(pub_topic, ExoCommand, queue_size=10)
        self._handler = handler
        self._widget = handler.widget
        self._data = data
        self._command_pub = publisher;
        self._command = ExoCommand()

        self._widget.setObjectName('BiomechGui')
        self._widget.startTrialButton.clicked.connect(self.start_trial)
        self._widget.endTrialButton.clicked.connect(self.end_trial)
        self._widget.calibrateTorqueButton.clicked.connect(self.calibrate_torque)
        self._widget.bluetoothCheckButton.clicked.connect(self.check_bluetooth)


    def send_command(self, command, *data):
        self._command.command_code = command
        self._command.data = data
        self._command_pub.publish(self._command)

    def report_info(self, info):
        self._widget.exoReportLabel.setText(info)

    def start_trial(self):
        self.report_info("Starting Trial")
        self.send_command(CommandCode.START_TRIAL)

    def end_trial(self):
        self.report_info("Ending Trial")
        self.send_command(CommandCode.END_TRIAL)

    def calibrate_torque(self):
        self.report_info("Calibrating Torque")
        self.send_command(CommandCode.CALIBRATE_TORQUE)

    def check_bluetooth(self):
        self._data["check_bluetooth_fail"] = True
        self.report_info("Checking bluetooth")
        self.send_command(CommandCode.CHECK_BLUETOOTH)
        rospy.Timer(rospy.Duration(1), self.check_bluetooth_fail, oneshot=True)

    def check_bluetooth_fail(self, event):
        if self._data["check_bluetooth_fail"]:
            self.report_info("Bluetooth failed")

    def get_pid(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_pid():
            self.send_command(CommandCode.GET_PID_PARAMS, identifier)
        return _get_pid

    def set_pid(self, widget, row, col):
        def _set_pid():
            try:
                p = float(widget.pidPLine.text())
                i = float(widget.pidILine.text())
                d = float(widget.pidDLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_PID_PARAMS, identifier, p,i,d)
            except ValueError:
                self.report_info("Bad Pid")
        return _set_pid

    def set_torque(self, widget, row, col):
        def _set_torque():
            try:
                pfx = float(widget.pfxLine.text())
                dfx = float(widget.dfxLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_TORQUE_SETPOINT, identifier, pfx,dfx)
            except ValueError:
                self.report_info("Bad Torque Setpoint")
        return _set_torque

    def get_torque(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_torque():
            self.send_command(CommandCode.GET_TORQUE_SETPOINT, identifier)
        return _get_torque
