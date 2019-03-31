#!/usr/bin/env python

import rospy
import struct
from biomech_comms.msg import ExoCommand
from std_msgs.msg import ByteMultiArray

def process_bytes_into_exo_data(msg, count, exoCommand):
    exoCommand.command_code = msg[2]
    print(msg[3:])
    msg_data = "".join([chr(c) for c in (msg[3:])])
    print(msg_data)
    data = struct.unpack("<f"*count, msg_data)
    exoCommand.data=data
    return exoCommand

class Transcriber:
    def __init__(self, pub):
        self.pub = pub
        self.bluetooth_buffer = []
        self.exoCommand = ExoCommand()

    def transcribe(self, data):
        self.bluetooth_buffer += data.data

        while True:
            try:
                start_index = self.bluetooth_buffer.index(ord('S'))
                count = self.bluetooth_buffer[start_index + 2]
                end_index = start_index + count*4 + 3
            except ValueError:
                break
            except IndexError:
                break
            msg = self.bluetooth_buffer[start_index:end_index]
            self.bluetooth_buffer = self.bluetooth_buffer[end_index + 1:]
            self.pub.publish(process_bytes_into_exo_data(msg, count, self.exoCommand))
        

def send_command_message(pub):
    byteArray = ByteMultiArray()
    byteArray.layout.dim = []
    byteArray.layout.data_offset = 0

    def _send_command_message(command):
        command_code = command.command_code
        data = command.data
        cmd_format = '<c' + "f"*len(data)
        cmd_bytes = struct.pack(cmd_format, chr(command_code).encode('utf-8'), *data)
        out = [ord(c) for c in cmd_bytes]
        byteArray.data = out
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
