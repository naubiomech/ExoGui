#!/usr/bin/env python
import rospy
from biomech.srv import GetBluetoothDevices, ConnectToBluetoothDevice

preferred_device = 'Jacks_Bluetooth' 

def setup_bluetooth():
    rospy.wait_for_service('/Exo/get_bluetooth_devices')
    print("Found service for getting bt devs")
    rospy.wait_for_service('/Exo/connect_to_bluetooth_device')
    print("service connecting")

    get_devices = rospy.ServiceProxy('/Exo/get_bluetooth_devices', GetBluetoothDevices)
    devices = get_devices()
    if preferred_device not in devices.devices:
        rospy.logerr("Failed to find {} in available devices".format(preferred_device))
        return

    connect_device = rospy.ServiceProxy('/Exo/connect_to_bluetooth_device', ConnectToBluetoothDevice)
    status = connect_device(preferred_device)
    if status.return_code != 0:
        rospy.logerr("Failed to connect to device: ({},{})".format(status.return_code, status.status_msg))


if __name__ == "__main__":
    try:
        setup_bluetooth()
    except rospy.ROSInterruptException:
        pass
    
