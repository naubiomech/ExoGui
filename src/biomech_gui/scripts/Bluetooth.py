#!/usr/bin/env python
import rospy
import bluetooth
from std_msgs.msg import ByteMultiArray, String
from std_srvs.srv import Empty
from biomech_gui.srv import GetBluetoothDevices, ConnectToBluetoothDevice

class Bluetooth:
    def __init__(self):
        self.port = 1
        self.sock = None
        self.devices = {}
        self.last_connected = None

    def get_bluetooth_devices(self, delay=3):
        devs_list = bluetooth.discover_devices(delay, lookup_names=True)
        devs = {host:address for address, host in devs_list}
        self.devices.update(devs)
        device_names = list(devs.keys())
        return [device_names]

    def connect_bluetooth_device(self, device_name):
        if device_name not in self.devices:
            return [1, "Device Not Recognized"]
        uuid = self.devices[device_name]
        return self.connect_bluetooth_device_uuid(uuid)

    def connect_bluetooth_device_uuid(self, uuid):
        try:
            if not self.sock:
                self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                self.sock.connect((uuid, self.port))
                self.last_connected = uuid
                return [0, "Connected"]
            else:
                return [2, "Device already connected"]
        except bluetooth.btcommon.BluetoothError as e:
            err = str(e)[1:-1].split(",")
            errno = int(err[0])
            strerror = err[1].strip()[1:-1]
            return [errno, strerror]

    def is_connected(self):
        return bool(self.sock)

    def reconnect(self):
        self.close()
        self.connect_bluetooth_device(self.last_connected)

    def send(self, byte_list):
        if self.is_connected():
            self.sock.send(byte_list)

    def receive(self, bufferSize=2048):
        if self.is_connected():
            return self._receive(bufferSize)
        return b''

    def _receive(self, bufferSize=2048):
        return self.sock.recv(bufferSize)

    def close(self):
        if self.is_connected():
            self.sock.close()
            self.sock = None

def disconnect_bluetooth(bt):
    def _disconnect_bluetooth(srv):
        bt.close()
        return []
    return try_safely(bt,_disconnect_bluetooth)

def scan_devices(bt):
    def _scan_devices(req):
        return bt.get_bluetooth_devices()
    return try_safely(bt,_scan_devices)

def connect_bluetooth(bt):
    def _connect_bluetooth(srv):
        return bt.connect_bluetooth_device(srv.device)
    return try_safely(bt,_connect_bluetooth)
    
def send_bluetooth_message(bt):
    def _send_bluetooth_message(data):
        byte_list = data.data
        bt.send(bytes(byte_list))
    return try_safely(bt, _send_bluetooth_message)

def try_safely(bt, func):
    def _safely(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            bt.close()
            raise e
    return _safely


def bluetooth_node():
    msg = ByteMultiArray()
    msg.layout.dim = []
    msg.layout.data_offset = 0
    bt = Bluetooth()

    pub = rospy.Publisher('bluetooth_rx', ByteMultiArray, queue_size=10)
    rospy.Subscriber('bluetooth_tx', ByteMultiArray, send_bluetooth_message(bt))
    rospy.Service('get_bluetooth_devices', GetBluetoothDevices, scan_devices(bt))
    rospy.Service('connect_to_bluetooth_device', ConnectToBluetoothDevice, connect_bluetooth(bt))
    rospy.Service('disconnect_from_bluetooth_device', Empty, disconnect_bluetooth(bt))

    rospy.init_node('Bluetooth', anonymous=True)
    rate = rospy.Rate(100000)

    try:
        while not rospy.is_shutdown():
            try:
                msg_bytes = bt.receive()
                if msg_bytes:
                    msg.data = msg_bytes
                    pub.publish(msg)
                rate.sleep()
            except bluetooth.btcommon.BluetoothError as e:
                err = str(e)[1:-1].split(",")
                errno = int(err[0])
                strerror = err[1].strip()[1:-1]
                if strerror == "Connection timed out":
                    rospy.logwarn("Bluetooth disconnection, Reconnecting...")
                    bt.reconnect()
                if strerror != "Interrupted system call":
                    rospy.logerr("Bluetooth Error: ({}, {})".format(errno,strerror))
                    break
    finally:
        bt.close()

if __name__ == "__main__":
    try:
        bluetooth_node()
    except rospy.ROSInterruptException:
        pass
