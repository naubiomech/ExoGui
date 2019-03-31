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
    def __init__(self, widget, sub_topic, data):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        subscriber = rospy.Subscriber(sub_topic, ExoCommand, self.process_data)
        self._widget = widget
        self._data = data
        self._command_sub = subscriber;
        self._callbacks = {}

        self.register(CommandCode.CHECK_BLUETOOTH, self.check_bluetooth)
        self.register(CommandCode.SET_LEFT_ANKLE_SETPOINT, self.get_torque)

    def register(self, exo_command, func):
        self._callbacks[exo_command] = func

    def report_info(self, info):
        self._widget.exoReportLabel.setText(info)
        
    def process_data(self, exo_command):
        self._callbacks[exo_command.command_code](*exo_command.data)

    def check_bluetooth(self, *data):
        valid_comm = data[0] == 0 and data[1] == 1 and data[2] == 2 
        valid = valid_comm and self._data["check_bluetooth_fail"] == True
        if valid:
            self._data["check_bluetooth_fail"] = False
            self.report_info("Bluetooth working")
        else:
            self.report_info("Bluetooth failed")

    def get_pid(self, p,i,d):
        pass

    def get_torque(self, widget):
        def _get_torque(self, pfx, dfx,):
            widget.PFXLabel.setText(pfx)
            widget.DFXLabel.setText(dfx)
        return _get_torque
