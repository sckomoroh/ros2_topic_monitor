from PyQt5 import QtWidgets

from main_window_ui import Ui_MainWindow

from monitor_widget import MonitorForm
from structure_widget import StructureForm
from topics_widget import TopicsWidget


class MainWindow(Ui_MainWindow, QtWidgets.QMainWindow):
    def __init__(self, parent, ros2_node, ros2_utils):
        super().__init__()
        self.setupUi(parent)

        self.widget_monitor = MonitorForm(self.monitor_tab, ros2_node)
        self.widget_structure = StructureForm(self.structure_tab, ros2_utils)
        self.widget_topics = TopicsWidget(self.topics_tab, ros2_utils)

        self.widget_topics.analyze_request.connect(
            self.widget_structure.on_analyze_requested
        )
        self.widget_structure.monitor_request.connect(
            self.widget_monitor.on_monitor_requested
        )

        self.monitor_vertical_layout = QtWidgets.QVBoxLayout(self.monitor_tab)
        self.structure_vertical_layout = QtWidgets.QVBoxLayout(self.structure_tab)

        self.monitor_vertical_layout.addWidget(self.widget_monitor)
        self.structure_vertical_layout.addWidget(self.widget_structure)

        self.widget_monitor.status_change.connect(self.__on_status_received)
        self.widget_structure.status_change.connect(self.__on_status_received)

    def __on_status_received(self, status):
        self.statusbar.showMessage(status, 1000)
