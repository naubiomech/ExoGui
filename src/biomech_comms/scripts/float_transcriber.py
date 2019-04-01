#!/usr/bin/env python

import rospy
import struct
import numpy as np
from biomech_comms.msg import ExoCommand
from std_msgs.msg import ByteMultiArray,MultiArrayDimension

def process_bytes_into_exo_data(msg, count, exoCommand):
    exoCommand.command_code = msg[1]
    data = np.array(msg[3:],dtype=np.byte).astype(np.ubyte)
    msg_data = "".join([chr(c) for c in data])
    data = struct.unpack("<"+"f"*count, msg_data)
    exoCommand.data=data
    return exoCommand

class Transcriber:
    def __init__(self, pub):
        self.pub = pub
        self.bluetooth_buffer = []
        self.exoCommand = ExoCommand()

    def transcribe(self, data):
        self.bluetooth_buffer += data.data

        try:
            start_index = self.bluetooth_buffer.index(ord('S'))
            count = self.bluetooth_buffer[start_index + 2]
            end_index = start_index + count*4 + 3
            if len(self.bluetooth_buffer) <= end_index:
                return
            msg = self.bluetooth_buffer[start_index:end_index]
        except ValueError:
            return
        except IndexError:
            return
        self.bluetooth_buffer = self.bluetooth_buffer[end_index + 1:]
        self.pub.publish(process_bytes_into_exo_data(msg, count, self.exoCommand))
        

def send_command_message(pub):
    byteArray = ByteMultiArray()
    byteArray.layout.dim.append(MultiArrayDimension())
    byteArray.layout.dim[0].label = 'Data'
    byteArray.layout.dim[0].size = 0
    byteArray.layout.dim[0].stride = 1
    byteArray.layout.data_offset = 0

    def _send_command_message(command):
        command_code = command.command_code
        data = command.data
        cmd_format = '<c' + "d"*len(data)
        cmd_bytes = struct.pack(cmd_format, chr(command_code).encode('utf-8'), *data)
        out = np.array([ord(c) for c in cmd_bytes],dtype=np.byte)
        byteArray.data = out
        byteArray.layout.dim[0].size = len(out)

        pub.publish(byteArray)

    return _send_command_message


        
def start_transcriber():
    rospy.init_node('transcriber', anonymous=True)

    output_pub = rospy.Publisher('exo_data', ExoCommand, queue_size=10)
    bt_pub = rospy.Publisher('bluetooth_tx', ByteMultiArray, queue_size=100)

    transcriber = Transcriber(output_pub)
    bt_sub = rospy.Subscriber('bluetooth_rx', ByteMultiArray, transcriber.transcribe) 
    input_sub = rospy.Subscriber('command_exo', ExoCommand, send_command_message(bt_pub))

    rospy.spin()


if __name__ == "__main__":
    start_transcriber()
