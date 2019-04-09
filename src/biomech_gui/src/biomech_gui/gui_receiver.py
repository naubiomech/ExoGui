#!/usr/bin/env python
import os
import time

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_plot.data_plot import DataPlot
from biomech_comms.msg import ExoCommand
from biomech_comms.comm_codes import CommandCode
from biomech_comms.joint_select import JointSelect

class ExoGuiReceiver():
    def __init__(self, handler, sub_topic, data):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        subscriber = rospy.Subscriber(sub_topic, ExoCommand, self.process_data)
        self._handler = handler
        self._widget = handler.widget
        self._data = data
        self._command_sub = subscriber;
        self._callbacks = {}

        self.register(CommandCode.CHECK_BLUETOOTH, self.check_bluetooth)

    def register(self, exo_command, func):
        self._callbacks[exo_command] = func

    def register_multi_widget(self, exo_command, identifier, func):
        if exo_command not in self._callbacks:
            self._callbacks[exo_command] = {}
        self._callbacks[exo_command][identifier] = func

    def report_info(self, info):
        self._widget.exoReportLabel.setText(info)
        
    def process_data(self, exo_command):
        print(exo_command.command_code)
        call = self._callbacks[exo_command.command_code]
        try:
            call(*exo_command.data)
        except TypeError:
            data = exo_command.data
            raw_identifier = self._handler.decode_msg_to_joint_select(data[0:3])
            print(call)
            call[raw_identifier](*data[3:])

    def updateLabel(self, label, num):
        label.setText("{:.2f}".format(num))



    def check_bluetooth(self, *data):
        valid_comm = data[0] == 0 and data[1] == 1 and data[2] == 2 
        valid = valid_comm and self._data["check_bluetooth_fail"] == True
        if valid:
            self._data["check_bluetooth_fail"] = False
            self.report_info("Bluetooth working")
        else:
            self.report_info("Bluetooth failed")

    def receive_pid(self,widget):
        def _receive_pid(p,i,d):
            self.updateLabel(widget.pidPLabel,p)
            self.updateLabel(widget.pidILabel,i)
            self.updateLabel(widget.pidDLabel,d)
        return _receive_pid

    def receive_smoothing(self,widget):
        def _receive_smoothing(n1,n2,n3):
            self.updateLabel(widget.n1Label,n1)
            self.updateLabel(widget.n2Label,n2)
            self.updateLabel(widget.n3Label,n3)
        return _receive_smoothing

    def receive_torque(self, widget):
        def _receive_torque(torque):
            self.updateLabel(widget.pfxLabel,torque)
            self.updateLabel(widget.dfxLabel,torque)
        return _receive_torque

    def receive_prop_gain(self, widget):
        def _receive_prop_gain(gain):
            self.updateLabel(widget.propGainLabel,gain)
        return _receive_prop_gain

    def receive_kf(self, widget):
        def _receive_kf(kf):
            self.updateLabel(widget.adjustKFLabel,kf)
        return _receive_kf

    def receive_fsr_thresh(self, widget):
        def _receive_fsr_thresh(fsr_thresh):
            self.updateLabel(widget.adjustFSRThLabel,fsr_thresh)
        return _receive_fsr_thresh
