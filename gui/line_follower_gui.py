#!/usr/bin/env python3
import sys
import subprocess
import signal
import rospy
from PyQt5 import QtWidgets, QtCore, QtGui
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist


class ColorIndicator(QtWidgets.QLabel):
    def __init__(self, diameter=30):
        super().__init__()
        self.diameter = diameter
        self.color = QtGui.QColor("gray")
        self.setFixedSize(diameter, diameter)

    def set_color(self, color_name):
        self.color = QtGui.QColor(color_name)
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setBrush(QtGui.QBrush(self.color))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(0, 0, self.diameter, self.diameter)


class LineFollowerGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        rospy.init_node("line_follower_gui", anonymous=True)

        try:
            self.sensors_process = subprocess.Popen(
                ["rosrun", "line_sens", "line_sensors_node"]
            )
        except Exception as e:
            print("[GUI] Error line_sensors_node:", e)
            self.sensors_process = None

        self.sensor_sub = rospy.Subscriber(
            "/line_sensors", Int32MultiArray, self.sensor_callback
        )

        self.cmd_sub = rospy.Subscriber(
            "/youbot_base/mecanum_drive_controller/cmd_vel", Twist, self.cmd_callback
        )

        self.setWindowTitle("Line Follower Controller")
        self.resize(420, 340)

        layout = QtWidgets.QVBoxLayout()

        # --------------------- Sensors section ---------------------
        group_sensors = QtWidgets.QGroupBox("Sensors")
        sensor_layout = QtWidgets.QGridLayout()

        font = QtGui.QFont()
        font.setPointSize(12)

        self.left_indicator = ColorIndicator(30)
        self.left_label = QtWidgets.QLabel("Left: ---")
        self.left_label.setFont(font)

        self.right_indicator = ColorIndicator(30)
        self.right_label = QtWidgets.QLabel("Right: ---")
        self.right_label.setFont(font)

        sensor_layout.addWidget(self.left_label,        0, 0)
        sensor_layout.addWidget(self.left_indicator,    0, 1)
        sensor_layout.addWidget(self.right_label,       1, 0)
        sensor_layout.addWidget(self.right_indicator,   1, 1)

        group_sensors.setLayout(sensor_layout)
        layout.addWidget(group_sensors)

        # ------------------ Direction section ------------------
        group_dir = QtWidgets.QGroupBox("Robot direction")
        dir_layout = QtWidgets.QVBoxLayout()

        self.direction_label = QtWidgets.QLabel("Direction: ---")
        self.direction_label.setFont(QtGui.QFont("Arial", 14))

        self.arrow_label = QtWidgets.QLabel("■")
        arrow_font = QtGui.QFont("Arial", 40, QtGui.QFont.Bold)
        self.arrow_label.setFont(arrow_font)
        self.arrow_label.setAlignment(QtCore.Qt.AlignCenter)

        dir_layout.addWidget(self.direction_label)
        dir_layout.addWidget(self.arrow_label)

        group_dir.setLayout(dir_layout)
        layout.addWidget(group_dir)

        # --------------------- Start/Stop ---------------------
        btn_layout = QtWidgets.QHBoxLayout()

        self.btn_start = QtWidgets.QPushButton("▶ Start robot")
        self.btn_stop  = QtWidgets.QPushButton("■ Stop robot")

        self.btn_start.setFont(font)
        self.btn_stop.setFont(font)

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)

        self.btn_start.clicked.connect(self.start_robot)
        self.btn_stop.clicked.connect(self.stop_robot)

        layout.addLayout(btn_layout)
        self.setLayout(layout)

        # ROS-friendly timer
        self.timer = QtCore.QTimer()
        self.timer.start(50)

        self.robot_process = None

        # Direction state
        self.current_direction = "stop"

    # -------------------- Callbacks --------------------
    def sensor_callback(self, msg):
        if len(msg.data) >= 2:
            left, right = msg.data[0], msg.data[1]

            self.left_label.setText(f"Left: {left}")
            self.right_label.setText(f"Right: {right}")

            self.left_indicator.set_color("black" if left == 0 else "green")
            self.right_indicator.set_color("black" if right == 0 else "green")

    def cmd_callback(self, msg: Twist):

        lin = msg.linear.x
        ang = msg.angular.z

        lin_th = 0.05
        ang_th = 0.1

        if abs(lin) > lin_th:
            direction = "forward"
        elif ang > ang_th:
            direction = "left"
        elif ang < -ang_th:
            direction = "right"
        else:
            direction = "stop"

        self.update_direction(direction)

    def update_direction(self, direction):
        self.current_direction = direction
        self.direction_label.setText(f"Direction: {direction}")

        if direction == "forward":
            self.arrow_label.setText("⬆")
        elif direction == "left":
            self.arrow_label.setText("⬅")
        elif direction == "right":
            self.arrow_label.setText("➡")
        else:
            self.arrow_label.setText("■")

    # -------------------- Process control --------------------
    def start_robot(self):
        if self.robot_process is None:
            try:
                self.robot_process = subprocess.Popen(
                    ["rosrun", "line_sens", "robot_control_node"]
                )
            except Exception as e:
                print("[GUI] Failed to start robot:", e)

    def stop_robot(self):
        if self.robot_process is not None:
            try:
                print("[GUI] Exit robot_control_node...")
                self.robot_process.send_signal(signal.SIGINT)
                self.robot_process.wait()
            except Exception as e:
                print("[GUI] Error stopping robot:", e)
            finally:
                self.robot_process = None

    def closeEvent(self, event):
        self.stop_robot()
        if self.sensors_process is not None:
            try:
                print("[GUI] Exit line_sensors_node...")
                self.sensors_process.send_signal(signal.SIGINT)
                self.sensors_process.wait()
            except:
                pass
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = LineFollowerGUI()
    gui.show()
    sys.exit(app.exec_())
    


if __name__ == "__main__":
    main()