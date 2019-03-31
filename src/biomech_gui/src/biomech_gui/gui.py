#!/usr/bin/env python
import os
import time

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGridLayout
from rqt_plot.data_plot import DataPlot
from biomech_gui.gui_sender import ExoGuiSender
from biomech_gui.gui_receiver import ExoGuiReceiver
from biomech_comms.comm_codes import CommandCode

class ExoControlPlugin(Plugin):
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(ExoControlPlugin, self).__init__(context)
        self.setObjectName("BiomechGuiPlugin")

        self._widget = QWidget()
        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('biomech_gui'), 'resource', 'gui_widget.ui')
        loadUi(ui_file, self._widget)

        data = {}
        sub_topic = 'exo_data'
        pub_topic = 'command_exo'
        self._sender = ExoGuiSender(self._widget, pub_topic, data)
        self._receiver = ExoGuiReceiver(self._widget, sub_topic, data)

        self.add_torque_widget(2,2,self._widget.torqueTab)

        context.add_widget(self._widget)


    def add_pid_widget(left_leg, right_leg):
        pass

    def add_torque_widget(self, left_leg, right_leg, parent):

        def _add_torque_widget(widget,row,col):
            widget.getTorqueButton.clicked.connect(self._sender.get_torque(widget,self._sender))
            widget.setTorqueButton.clicked.connect(self._sender.set_torque(widget))
            self._receiver.register(CommandCode.SET_LEFT_ANKLE_SETPOINT, self._receiver.get_torque(widget))

        layout = QGridLayout()
        parent.setLayout(layout)
        ui_file = os.path.join(self.rp.get_path('biomech_gui'), 'resource', 'torque_gui.ui')
        data = {}
        self._widget.torque_widgets = data
        self.add_widgets(layout, ui_file,data,left_leg,right_leg,_add_torque_widget)
                
    def add_widgets(self, parent, ui_file,data,
                    left_leg_count, right_leg_count, setup_callback):
        self._widget.torque_widgets["left"] = []
        for i in range(left_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            self._widget.torque_widgets["left"].append(widget)
            parent.addWidget(widget,i,0)
            setup_callback(widget,row=i,col=0)
                
        self._widget.torque_widgets["right"] = []
        for i in range(right_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            self._widget.torque_widgets["right"].append(widget)
            parent.addWidget(widget,i,1)
            setup_callback(widget,row=i,col=1)
