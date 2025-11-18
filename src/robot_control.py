#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import re

class RobotControlNode:
    def __init__(self):
        rospy.init_node('robot_control_node')
        self.sub = rospy.Subscriber('/line_sensors', Int32MultiArray, self.callback)
        self.pub = rospy.Publisher('/youbot_base/mecanum_drive_controller/cmd_vel', Twist, queue_size=1)

        self.left_sens = 2
        self.right_sens = 2
        self.base_speed = 0.08
        self.turn_koeff = 0.25
        self.direction = -1

    def callback(self, data):
        time.sleep(0.01)
        self.left_sens = data.data[0]
        self.right_sens = data.data[1]

        if (self.left_sens == 1 and self.right_sens == 1):
            angular_z = 0
            linear_x = 1
        elif (self.left_sens == 0 and self.right_sens == 1):
            angular_z = -1
            linear_x = 0
        elif (self.left_sens == 1 and self.right_sens == 0):
            angular_z = 1
            linear_x = 0
        else:
            angular_z = 0
            linear_x = 0
        # error = self.left_sens - self.right_sens

        # angular_z = self.turn_koeff * error
        # linear_x = -self.base_speed

        # angular_z = max(min(angular_z, 1.0), -1.0)

        cmd = Twist()
        cmd.linear.x = self.direction * self.base_speed * linear_x
        cmd.angular.z = self.direction * self.turn_koeff * angular_z

        # Отправляем команду
        self.pub.publish(cmd)

        rospy.loginfo(f"left={self.left_sens} right={self.right_sens} turn={angular_z}")
            

if __name__ == '__main__':
        node = RobotControlNode()
        rate = rospy.Rate(10)

        while(not rospy.is_shutdown()):
            try:
                print("data: ", node.left_sens, node.right_sens)
                rate.sleep()

            except rospy.ROSInterruptException:
                pass
            except Exception as e:
                print(f"An error occurred: {e}")