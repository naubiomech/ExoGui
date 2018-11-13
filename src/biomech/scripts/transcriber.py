#!/usr/bin/env python

import rospy
import struct
from biomech.msg import ExoCommand
from std_msgs.msg import ByteMultiArray

def process_bytes_into_exo_data(byte_msg, exoCommand):
    msg = bytes(byte_msg).decode('utf-8').split(',')
    exoCommand.command_code = ord(msg[0])
    data = [float(data) for data in msg[1:-1]]
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
                end_index = self.bluetooth_buffer[start_index:].index(ord('\n')) + start_index
            except ValueError:
                break
            msg = self.bluetooth_buffer[start_index:end_index]
            self.bluetooth_buffer = self.bluetooth_buffer[end_index + 1:]
            self.pub.publish(process_bytes_into_exo_data(msg, self.exoCommand))
        

def send_command_message(pub):
    byteArray = ByteMultiArray()
    byteArray.layout.dim = []
    byteArray.layout.data_offset = 0

    def _send_command_message(command):
        command_code = command.command_code
        data = command.data
        cmd_bytes = struct.pack('<c' + "d"*len(data), chr(command_code).encode('utf-8'), *data)
        byteArray.data = cmd_bytes
        pub.publish(byteArray)

    return _send_command_message


        
def start_transcriber():
    rospy.init_node('transcriber', anonymous=True)

    output_pub = rospy.Publisher('exo_data', ExoCommand, queue_size=10)
    bt_pub = rospy.Publisher('bluetooth_tx', ByteMultiArray, queue_size=10)

    transcriber = Transcriber(output_pub)
    bt_sub = rospy.Subscriber('bluetooth_rx', ByteMultiArray, transcriber.transcribe) 
    input_sub = rospy.Subscriber('command_exo', ExoCommand, send_command_message(bt_pub))

    rospy.spin()


if __name__ == "__main__":
    start_transcriber()
