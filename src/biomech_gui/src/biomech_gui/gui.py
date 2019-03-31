#!/usr/bin/env python
import os
import time

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_plot.data_plot import DataPlot
from biomech_gui.gui_sender import ExoGuiSender
from biomech_gui.gui_receiver import ExoGuiReceiver

class ExoControlPlugin(Plugin):
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(ExoControlPlugin, self).__init__(context)
        self.setObjectName("BiomechGuiPlugin")

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('biomech_gui'), 'resource', 'gui_widget.ui')
        loadUi(ui_file, self._widget)

        data = {}
        sub_topic = 'exo_data'
        pub_topic = 'command_exo'
        self._sender = ExoGuiSender(self._widget, pub_topic, data)
        self._receiver = ExoGuiReceiver(self._widget, sub_topic, data)

        context.add_widget(self._widget)
