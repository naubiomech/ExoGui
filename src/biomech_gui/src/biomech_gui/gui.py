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
        self._widget.startTrialButton.clicked.connect(self.start_trial)
        self._widget.endTrialButton.clicked.connect(self.end_trial)
        context.add_widget(self._widget)
        self.command_pub = rospy.Publisher('command_exo', ExoCommand, queue_size=10)
        self.command = ExoCommand()


    def report_info(self, info):
        self._widget.exoReportLabel.setText(info)

    def start_trial(self):
        self.report_info("Starting Trial")
        self.command.command_code = CommandCode.START_TRIAL
        self.command.data = []
        self.command_pub.publish(self.command)

    def end_trial(self):
        self.report_info("Ending Trial")
        self.command.command_code = CommandCode.END_TRIAL
        self.command.data = []
        self.command_pub.publish(self.command)
