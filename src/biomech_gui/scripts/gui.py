#!/usr/bin/env python
import os
import time

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QApplication

class ExoControlWidget(QApplication):
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(ExoControlWidget, self).__init__(context)
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('biomech_gui'), 'resource', 'ros_gui.ui')
        loadUi(ui_file, self, {})

        self.setObjectName('ExoControl')

