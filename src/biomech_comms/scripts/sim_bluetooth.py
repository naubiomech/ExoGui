#!/usr/bin/env python

import rospy
from std_msgs.msg import ByteMultiArray, Int16
from std_srvs.srv import Empty
from biomech_comms.srv import GetBluetoothDevices, ConnectToBluetoothDevice

def disconnect_bluetooth():
    def _disconnect_bluetooth(srv):
        return []
    return _disconnect_bluetooth

def scan_devices():
    def _scan_devices(req):
        return [["simulated"]]
    return _scan_devices

def connect_bluetooth():
    def _connect_bluetooth(srv):
        return [0, "Connected"]
    return _connect_bluetooth

def pass_out_message(pub):
    out_msg = Int16()

    def _pass_out_message(data):
        for data_int in data.data:
            out_msg.data = data_int
            pub.publish(out_msg)

    return _pass_out_message

def pass_into_message(pub):
    out_msg = ByteMultiArray()
    out_msg.layout.dim = []
    out_msg.layout.data_offset = 0
    def _pass_into_message(data):
        msg_int = int(data.data)
        out_msg.data = [msg_int]
        pub.publish(out_msg)

    return _pass_into_message

def bluetooth_node():
    rx_port = 'Pin0'
    tx_port = 'Pin1'

    bt_pub = rospy.Publisher('bluetooth_rx', ByteMultiArray, queue_size=100)
    port_pub = rospy.Publisher(tx_port, Int16, queue_size=100)
    bt_sub = rospy.Subscriber('bluetooth_tx', ByteMultiArray, pass_out_message(port_pub))
    port_sub = rospy.Subscriber(rx_port, Int16, pass_into_message(bt_pub))

    rospy.Service('get_bluetooth_devices', GetBluetoothDevices, scan_devices())
    rospy.Service('connect_to_bluetooth_device', ConnectToBluetoothDevice, connect_bluetooth())
    rospy.Service('disconnect_from_bluetooth_device', Empty, disconnect_bluetooth())

    rospy.init_node('Simulated_Bluetooth', anonymous=True)
    rospy.spin()

if __name__ == "__main__":
    try:
        bluetooth_node()
    except rospy.ROSInterruptException:
        pass
