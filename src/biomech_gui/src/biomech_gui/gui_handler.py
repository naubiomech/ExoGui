import os
import time
import rospkg

from biomech_comms.comm_codes import CommandCode

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGridLayout

class ExoGuiHandler:
    def __init__(self, widget):
        self.widget = widget

        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('biomech_gui'), 'resource', 'gui_widget.ui')
        loadUi(ui_file, self.widget)

        self.widget.exoVersionLabel.setText("4.0.0")
        pass
    
    def add_pid_widget(self, left_leg, right_leg):
        pass

    def add_torque_widget(self, left_leg, right_leg, parent):

        def _add_torque_widget(widget,row,col):
            widget.getTorqueButton.clicked.connect(self._sender.get_torque(widget,self._sender))
            widget.setTorqueButton.clicked.connect(self._sender.set_torque(widget))
            self._receiver.register(CommandCode.GET_LEFT_ANKLE_SETPOINT, self._receiver.get_torque(widget))

        layout = QGridLayout()
        parent.setLayout(layout)
        ui_file = os.path.join(self.rp.get_path('biomech_gui'), 'resource', 'torque_gui.ui')
        data = {}
        self.widget.torque_widgets = data
        self.add_widgets(layout, ui_file,data,left_leg,right_leg,_add_torque_widget)
                
    def add_widgets(self, parent, ui_file,data,
                    left_leg_count, right_leg_count, setup_callback):
        self.widget.torque_widgets["left"] = []
        for i in range(left_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            self.widget.torque_widgets["left"].append(widget)
            parent.addWidget(widget,i,0)
            setup_callback(widget,row=i,col=0)
                
        self.widget.torque_widgets["right"] = []
        for i in range(right_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            self.widget.torque_widgets["right"].append(widget)
            parent.addWidget(widget,i,1)
            setup_callback(widget,row=i,col=1)
