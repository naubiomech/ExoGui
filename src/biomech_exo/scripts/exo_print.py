#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int16

class Printer:
    def __init__(self,pub):
        self.byte_buffer = []
        self.pub = pub

    def send_command_message(self,data):
        self.byte_buffer += [data.data]
        while True:
            try:
                end_index = self.byte_buffer.index(10)
                string = self.byte_buffer[:end_index+1]
                self.byte_buffer = self.byte_buffer[end_index+1:]
                string = str("".join([chr(c) for c in string]))
                self.pub.publish(string)
            except ValueError:
                break


        
def start_printer():
    rospy.init_node('transcriber', anonymous=True)
    rx_pin = 'Pin100'

    output_pub = rospy.Publisher('/exo_printing', String, queue_size=10)
    printer = Printer(output_pub)
    input_sub = rospy.Subscriber(rx_pin, Int16, printer.send_command_message)

    rospy.spin()


if __name__ == "__main__":
    start_printer()
