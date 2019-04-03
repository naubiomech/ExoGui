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

    def add_torque_widget(self, left_leg, right_leg, parent, receive_register,
                          get_torque, set_torque, receive_torque):

        def _add_torque_widget(widget,row,col):
            widget.getTorqueButton.clicked.connect(get_torque(widget, identifier))
            widget.setTorqueButton.clicked.connect(set_torque(widget, identifier))
            receive_register(CommandCode.GET_LEFT_ANKLE_SETPOINT, receive_torque(widget))

        torque_widgets = {}
        self.widget.torque_widgets = torque_widgets
        layout, ui_file = self.prepare_widget_parent_grid(parent, 'torque_gui.ui')
        self.add_widgets(layout, ui_file,data,left_leg,right_leg,_add_torque_widget)

    def prepare_widget_parent_grid(self, parent, filename):
        layout = QGridLayout()
        parent.setLayout(layout)
        ui_file = os.path.join(self.rp.get_path('biomech_gui'), 'resource', filename)
        return layout, ui_file
                
    def add_widgets(self, parent, ui_file, data,
                    left_leg_count, right_leg_count, setup_callback):
        data["left"] = []
        for i in range(left_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            data["left"].append(widget)
            parent.addWidget(widget,i,0)
            setup_callback(widget,row=i,col=0)
                
        data["right"] = []
        for i in range(right_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            data["right"].append(widget)
            parent.addWidget(widget,i,1)
            setup_callback(widget,row=i,col=1)
