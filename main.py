#!/bin/python

from PyQt5.QtWidgets import QApplication, QMainWindow
from main_window import MainWindow
import rclpy

from ros2_node import Ros2Node


class MonitorMainWindow(QMainWindow):
    def __init__(self, node: Ros2Node):
        super().__init__()
        self.node = node

    def closeEvent(self, event):
        self.node.stop_node()
        rclpy.shutdown()
        event.accept()


def main():
    rclpy.init(args=None)
    node = Ros2Node()
    node.start_node()

    app = QApplication([])
    mainWindow = MonitorMainWindow(node)
    window = MainWindow(mainWindow, node, node.ros2_utils)
    mainWindow.show()
    app.exec_()


if __name__ == "__main__":
    main()
