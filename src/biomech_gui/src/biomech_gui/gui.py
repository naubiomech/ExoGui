#!/usr/bin/env python

import rospy

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QGridLayout
from rqt_plot.data_plot import DataPlot
from biomech_gui.gui_sender import ExoGuiSender
from biomech_gui.gui_receiver import ExoGuiReceiver
from biomech_gui.gui_handler import ExoGuiHandler

class ExoControlPlugin(Plugin):
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(ExoControlPlugin, self).__init__(context)
        self.setObjectName("BiomechGuiPlugin")

        self._widget = QWidget()
        
        data = {}
        sub_topic = 'exo_data'
        pub_topic = 'command_exo'
        self._handler = ExoGuiHandler(self._widget)
        self._sender = ExoGuiSender(self._handler, pub_topic, data)
        self._receiver = ExoGuiReceiver(self._handler, sub_topic, data)

        left_leg_count = 2
        right_leg_count = 2
        self._handler.add_torque_widgets(self._widget.torqueTab, left_leg_count, right_leg_count,
                                         self._sender, self._receiver)
        self._handler.add_pid_widgets(self._widget.pidTab, left_leg_count, right_leg_count,
                                      self._sender, self._receiver)
        self._handler.add_adjust_widgets(self._widget.adjustTab, left_leg_count, right_leg_count,
                                      self._sender, self._receiver)

        context.add_widget(self._widget)


