#!/usr/bin/env python3

import serial
import rospy
from std_msgs.msg import Int32MultiArray
import re

class LineSensorNode:
    def __init__(self):
        rospy.init_node('line_sensor_node')
        self.pub = rospy.Publisher('/line_sensors', Int32MultiArray, queue_size=1)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    def read_serial(self):
        line = self.ser.readline().decode().strip()
        if not line:
            return
        try:
            values = re.findall(r"first=(\d), second=(\d)", line)
            left, right = map(int, values[0])
            msg = Int32MultiArray()
            msg.data = [left, right]
            self.pub.publish(msg)
        except ValueError:
            print(f'Invalid data: {line}')


if __name__ == '__main__':
        node = LineSensorNode()
        rate = rospy.Rate(10)

        while(not rospy.is_shutdown()):
            try:
                node.read_serial()
                rate.sleep()

            except serial.SerialException as e:
                print(f"Serial port error: {e}")
            except rospy.ROSInterruptException:
                pass
            except Exception as e:
                print(f"An error occurred: {e}")
