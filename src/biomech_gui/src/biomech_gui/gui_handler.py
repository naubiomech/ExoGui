import os
import time
import rospkg

from biomech_comms.comm_codes import CommandCode
from biomech_comms.joint_select import JointSelect

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGridLayout

class ExoGuiHandler:
    def __init__(self, widget):
        self.widget = widget

        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('biomech_gui'), 'resource', 'gui_widget.ui')
        loadUi(ui_file, self.widget)

        self.widget.exoVersionLabel.setText("4.0.0")

    def add_pid_widgets(self, parent, left_leg, right_leg, sender, receiver):
        widgetCount = (left_leg, right_leg)
        getData = ("pidGetButton", sender.get_pid)
        setData = ("pidSetButton", sender.set_pid)
        self.widget.pid_widgets = {}
        widgetData = (self.widget.pid_widgets, "pid_gui.ui")
        receiveData = (receiver.register_multi_widget, CommandCode.GET_PID_PARAMS, receiver.receive_pid)
        self.add_set_get_receive_widget(parent, widgetCount, widgetData, getData, setData, receiveData) 
    
    def add_torque_widgets(self, parent, left_leg, right_leg, sender, receiver):
        widgetCount = (left_leg, right_leg)
        getData = ("getTorqueButton", sender.get_torque)
        setData = ("setTorqueButton", sender.set_torque)
        self.widget.torque_widgets = {}
        widgetData = (self.widget.torque_widgets, "torque_gui.ui")
        receiveData = (receiver.register_multi_widget, CommandCode.GET_TORQUE_SETPOINT, receiver.receive_torque)
        self.add_set_get_receive_widget(parent, widgetCount, widgetData, getData, setData, receiveData) 
    
    def add_set_get_receive_widget(self, parent, widgetCounts, widgetData, getData, setData, receiveData):

        def _add_widget(widget,row,col):
            identifier = self.decode_widget_to_joint_select_msg(row, col)
            getattr(widget,getData[0]).clicked.connect(getData[1](widget,row,col))
            getattr(widget,setData[0]).clicked.connect(setData[1](widget,row,col))
            receiveData[0](receiveData[1], identifier, receiveData[2](widget))

        layout, ui_file = self.prepare_widget_parent_grid(parent, widgetData[1])
        self.add_widgets(layout, ui_file, widgetData[0],widgetCounts,_add_widget)

    def prepare_widget_parent_grid(self, parent, filename):
        layout = QGridLayout()
        parent.setLayout(layout)
        ui_file = os.path.join(self.rp.get_path('biomech_gui'), 'resource', filename)
        return layout, ui_file
                
    def add_widgets(self, parent, ui_file, data,
                    widgetCounts, setup_callback):
        left_leg_count = widgetCounts[0]
        data["left"] = []
        for i in range(left_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            data["left"].append(widget)
            parent.addWidget(widget,left_leg_count - 1 - i,0)
            setup_callback(widget,row=i,col=0)
                
        right_leg_count = widgetCounts[1]
        data["right"] = []
        for i in range(right_leg_count):
            widget = QWidget()
            loadUi(ui_file, widget)
            data["right"].append(widget)
            parent.addWidget(widget,right_leg_count - 1 - i,1)
            setup_callback(widget,row=i,col=1)

    def decode_widget_to_joint_select(self, row, col):
        area = col
        joint = row
        state = 3
        return area, joint, state
        

    def decode_widget_to_joint_select_msg(self, row, col):
        area,joint,state = self.decode_widget_to_joint_select(row,col)
        int_identifer = JointSelect.select_joint(area,joint,state)
        return JointSelect.encode_select_to_msg_double(int_identifer)

