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

    def get_kf(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_kf():
            self.send_command(CommandCode.GET_KF, *identifier)
        return _get_kf

    def set_kf(self, widget, row, col):
        def _set_kf():
            try:
                kf = float(widget.adjustKFLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_KF, identifier, kf)
            except ValueError:
                self.report_info("Bad Kf")
        return _set_kf

    def get_prop_gain(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_gain():
            self.send_command(CommandCode.GET_GAIN, *identifier)
        return _get_gain

    def set_prop_gain(self, widget, row, col):
        def _set_gain():
            try:
                gain = float(widget.propGainLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_GAIN, identifier[0], identifier[1], identifier[2], gain)
            except ValueError:
                self.report_info("Bad Gain")
        return _set_gain

    def get_smoothing(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_smoothing():
            self.send_command(CommandCode.GET_SMOOTHING_PARAMS, identifier[0], identifier[1], identifier[2])
        return _get_smoothing

    def set_smoothing(self, widget, row, col):
        def _set_smoothing():
            try:
                n1 = float(widget.n1setLine.text())
                n2 = float(widget.n2setLine.text())
                n3 = float(widget.n3setLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_SMOOTHING_PARAMS, identifier[0], identifier[1], identifier[2], n1, n2, n3)
            except ValueError:
                self.report_info("Bad Smoothing")
        return _set_smoothing

    def get_fsr_thresh(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_fsr_thresh():
            self.send_command(CommandCode.GET_FSR_THRESHOLD, identifier[0], identifier[1], identifier[2])
        return _get_fsr_thresh

    def set_fsr_thresh(self, widget, row, col):
        def _set_fsr_thresh():
            try:
                fsr_thresh = float(widget.adjustFSRThLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_FSR_THRESHOLD, identifier[0], identifier[1], identifier[2], fsr_thresh)
            except ValueError:
                self.report_info("Bad Fsr_Thresh")
        return _set_fsr_thresh

    def get_pid(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_pid():
            self.send_command(CommandCode.GET_PID_PARAMS, identifier[0], identifier[1], identifier[2])
        return _get_pid

    def set_pid(self, widget, row, col):
        def _set_pid():
            try:
                p = float(widget.pidPLine.text())
                i = float(widget.pidILine.text())
                d = float(widget.pidDLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_PID_PARAMS, identifier[0], identifier[1], identifier[2], p,i,d)
            except ValueError:
                self.report_info("Bad Pid")
        return _set_pid

    def set_torque(self, widget, row, col):
        def _set_torque():
            try:
                pfx = float(widget.pfxLine.text())
                dfx = float(widget.dfxLine.text())
                identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
                self.send_command(CommandCode.SET_TORQUE_SETPOINT, identifier[0], identifier[1], identifier[2], pfx,dfx)
            except ValueError:
                self.report_info("Bad Torque Setpoint")
        return _set_torque

    def get_torque(self, widget, row, col):
        identifier = self._handler.decode_widget_to_joint_select_msg(row=row, col=col)
        def _get_torque():
            self.send_command(CommandCode.GET_TORQUE_SETPOINT, identifier[0], identifier[1], identifier[2])
        return _get_torque
