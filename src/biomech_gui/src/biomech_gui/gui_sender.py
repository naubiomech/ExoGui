#!/usr/bin/env python
import os
import time

import rospy

from biomech_comms.msg import ExoCommand
from biomech_comms.comm_codes import CommandCode

class ExoGuiSender():
    def __init__(self, widget, pub_topic, data):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        publisher = rospy.Publisher(pub_topic, ExoCommand, queue_size=10)

        self._widget = widget
        self._data = data
        self._command_pub = publisher;
        self._command = ExoCommand()

        self._widget.setObjectName('BiomechGui')
        self._widget.startTrialButton.clicked.connect(self.start_trial)
        self._widget.endTrialButton.clicked.connect(self.end_trial)
        self._widget.calibrateTorqueButton.clicked.connect(self.calibrate_torque)


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
