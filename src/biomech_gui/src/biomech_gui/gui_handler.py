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

    def add_adjust_widgets(self, parent, left_leg, right_leg, sender, receiver):
        self.widget.pid_widgets = {}
        widgetCount = (left_leg, right_leg)
        widgetData = (self.widget.pid_widgets, "adjust_gui.ui", "adjustBox")

        getKFData = ("adjustGetKFButton", sender.get_kf)
        setKFData = ("adjustSetKFButton", sender.set_kf)
        getFSRThData = ("adjustFSRThGetButton", sender.get_kf)
        setFSRThData = ("adjustFSRThSetButton", sender.set_kf)
        sending = [getKFData, setKFData, getFSRThData, setFSRThData]

        receiveKFData = (CommandCode.GET_KF, receiver.receive_pid)
        receiveFSRThData = (CommandCode.GET_FSR_THRESHOLD, receiver.receive_fsr_thresh)
        receiving = [receiveKFData, receiveFSRThData]

        self.add_set_get_receive_widget(parent, widgetCount, widgetData,
                                        receiver.register_multi_widget, sending, receiving) 
    
    def add_pid_widgets(self, parent, left_leg, right_leg, sender, receiver):
        self.widget.pid_widgets = {}
        widgetCount = (left_leg, right_leg)
        widgetData = (self.widget.pid_widgets, "pid_gui.ui", "pidBox")

        getData = ("pidGetButton", sender.get_pid)
        setData = ("pidSetButton", sender.set_pid)
        sending = [getData, setData]

        receiveData = (CommandCode.GET_PID_PARAMS, receiver.receive_pid)
        receiving = [receiveData]

        self.add_set_get_receive_widget(parent, widgetCount, widgetData, receiver.register_multi_widget, sending, receiving) 

    def add_prop_widgets(self, parent, left_leg, right_leg, sender, receiver):
        self.widget.prop_widgets = {}
        widgetCount = (left_leg, right_leg)
        widgetData = (self.widget.prop_widgets, "proportional_control_gui.ui", "propBox")

        getData = ("propGetGainButton", sender.get_prop_gain)
        setData = ("propSetGainButton", sender.set_prop_gain)
        sending = [getData, setData]

        receiveData = (CommandCode.GET_GAIN, receiver.receive_prop_gain)
        receiving = [receiveData]

        self.add_set_get_receive_widget(parent, widgetCount, widgetData, receiver.register_multi_widget, sending, receiving) 

    def add_smoothing_widgets(self, parent, left_leg, right_leg, sender, receiver):
        self.widget.prop_widgets = {}
        widgetCount = (left_leg, right_leg)
        widgetData = (self.widget.prop_widgets, "smoothing_gui.ui", "smoothingBox")

        getData = ("getSmoothing", sender.get_smoothing)
        setData = ("setSmoothing", sender.set_smoothing)
        sending = [getData, setData]

        receiveData = (CommandCode.GET_SMOOTHING_PARAMS, receiver.receive_smoothing)
        receiving = [receiveData]

        self.add_set_get_receive_widget(parent, widgetCount, widgetData, receiver.register_multi_widget, sending, receiving) 
    
    def add_torque_widgets(self, parent, left_leg, right_leg, sender, receiver):
        self.widget.torque_widgets = {}
        widgetCount = (left_leg, right_leg)
        widgetData = (self.widget.torque_widgets, "torque_gui.ui", "torqueBox")

        getData = ("getTorqueButton", sender.get_torque)
        setData = ("setTorqueButton", sender.set_torque)
        sending = [getData, setData]

        receiveData = (CommandCode.GET_TORQUE_SETPOINT, receiver.receive_torque)
        receiving = [receiveData]

        self.add_set_get_receive_widget(parent, widgetCount, widgetData, receiver.register_multi_widget, sending, receiving) 
    
    def add_set_get_receive_widget(self, parent, widgetCounts, widgetData, receiver_register, sendingData, receivingData):
        area_name = ["Left","Right"]
        joint_name = ["Ankle", "Knee"]
        def _add_widget(widget,row,col):
            selects = self.decode_widget_to_joint_select(row, col)
            identifier = self.decode_widget_to_joint_select_msg(row, col)
            title = "{} {}".format(area_name[col], joint_name[row])
            getattr(widget,widgetData[2]).setTitle(title)
            for data in sendingData:
                getattr(widget,data[0]).clicked.connect(data[1](widget,row,col))
            for data in receivingData:
                receiver_register(data[0], selects, data[1](widget))

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
        return (area, joint, state)
        

    def decode_widget_to_joint_select_msg(self, row, col):
        area,joint,state = self.decode_widget_to_joint_select(row,col)
        int_identifer = JointSelect.select_joint(area,joint,state)
        return JointSelect.encode_select_to_msg_double(int_identifer)

    def decode_msg_to_joint_select(self, msg):
        int_identifier = JointSelect.decode_msg_to_joint_select(msg)
        return JointSelect.unselect_joint(int_identifier)
