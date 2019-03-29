#!/usr/bin/env python

import rospy
from std_msgs.msg import ByteMultiArray, String
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

def pass_message(pub):
    def _pass_message(data):
        pub.publish(data)
    return _pass_message

def bluetooth_node():
    msg = ByteMultiArray()
    msg.layout.dim = []
    msg.layout.data_offset = 0

    rx_port = 'Pin0'
    tx_port = 'Pin1'

    bt_pub = rospy.Publisher('bluetooth_rx', ByteMultiArray, queue_size=10)
    port_pub = rospy.Publisher(rx_port, ByteMultiArray, queue_size=10)
    bt_sub = rospy.Subscriber('bluetooth_tx', ByteMultiArray, pass_message(port_pub))
    port_sub = rospy.Subscriber(tx_port, ByteMultiArray, pass_message(bt_pub))

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
