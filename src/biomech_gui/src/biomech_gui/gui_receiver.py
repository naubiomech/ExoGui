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
        call = self._callbacks[exo_command.command_code](*exo_command.data)
        try:
            call(*exo_command.data)
        except TypeError:
            data = exo_command.data
            raw_identifier = data[0]
            call[raw_identifier](*data[1:])

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
            widget.pidPLabel.setText(str(p))
            widget.pidILabel.setText(str(i))
            widget.pidDLabel.setText(str(d))
        return _receive_pid

    def receive_smoothing(self,widget):
        def _receive_smoothing(n1,n2,n3):
            widget.n1Label.setText(str(n1))
            widget.n2Label.setText(str(n2))
            widget.n3Label.setText(str(n3))
        return _receive_smoothing

    def receive_torque(self, widget):
        def _receive_torque(torque):
            torque = str(torque)
            widget.PFXLabel.setText(torque)
            widget.DFXLabel.setText(torque)
        return _receive_torque

    def receive_prop_gain(self, widget):
        def _receive_prop_gain(gain):
            gain = str(gain)
            widget.propGainLabel.setText(gain)
        return _receive_prop_gain

    def receive_kf(self, widget):
        def _receive_kf(kf):
            kf = str(kf)
            widget.adjustKFLabel.setText(kf)
        return _receive_kf

    def receive_fsr_thresh(self, widget):
        def _receive_fsr_thresh(fsr_thresh):
            fsr_thresh = str(fsr_thresh)
            widget.adjustFSRThLabel.setText(fsr_thresh)
        return _receive_fsr_thresh
