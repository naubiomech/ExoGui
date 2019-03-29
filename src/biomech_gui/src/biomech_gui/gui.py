#!/usr/bin/env python
import os
import time

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_plot.data_plot import DataPlot

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
        self._widget.setObjectName('BiomechGui')
        context.add_widget(self._widget)

