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

def pass_into_message(pub):
    out_msg = Int16()

    def _pass_into_message(data):
        for data_int in data.data:
            out_msg.data = data_int
            pub.publish(out_msg)

    return _pass_into_message

def pass_out_message(pub):
    out_msg = ByteMultiArray()
    def _pass_out_message(data):
        msg_int = int(data.data)
        out_msg.data = bytes(msg_int)
        pub.publish(out_msg)

    return _pass_out_message

def bluetooth_node():
    msg = ByteMultiArray()
    msg.layout.dim = []
    msg.layout.data_offset = 0

    rx_port = 'Pin1'
    tx_port = 'Pin0'

    bt_pub = rospy.Publisher('bluetooth_rx', ByteMultiArray, queue_size=10)
    port_pub = rospy.Publisher(rx_port, Int16, queue_size=10)
    bt_sub = rospy.Subscriber('bluetooth_tx', ByteMultiArray, pass_into_message(port_pub))
    port_sub = rospy.Subscriber(tx_port, Int16, pass_out_message(bt_pub))

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
