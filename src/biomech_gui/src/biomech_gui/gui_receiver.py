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

class ExoGuiReceiver():
    def __init__(self, widget, sub_topic, data):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        subscriber = rospy.Subscriber(sub_topic, ExoCommand, queue_size=10)
        self._widget = widget
        self._data = data
        self._command_sub = subscriber;
        self._callbacks = {}

    def register(self, exo_command, func):
        self._callbacks[exo_command] = func
        
    def process_data(self, exo_command):
        self._callbacks[exo_command.command_code](*exo_command.data)
