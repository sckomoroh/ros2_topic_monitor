from topics_widget_ui import Ui_TopicsWidget
from ros2_node import Ros2Utils
from PyQt5.QtCore import QAbstractListModel, Qt, pyqtSignal, QSortFilterProxyModel
from PyQt5.QtWidgets import QMenu, QWidget


class RunningNodesListModel(QAbstractListModel):
    def __init__(self, parent):
        super(RunningNodesListModel, self).__init__(parent)
        self.data = []

    def rowCount(self, parent=None):
        return len(self.data)

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid() and role == Qt.DisplayRole:
            return self.data[index.row()]
        return None

    def set_list(self, list):
        self.beginResetModel()

        self.data = list

        self.endResetModel()


class TopicsWidget(Ui_TopicsWidget, QWidget):
    analyze_request = pyqtSignal(str)

    def __init__(self, parent, ros2_utils: Ros2Utils):
        super().__init__()
        self.setupUi(parent)

        self.ros2_utils = ros2_utils

        self.model = RunningNodesListModel(self)
        self.proxy_model = QSortFilterProxyModel()
        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setFilterCaseSensitivity(Qt.CaseInsensitive)

        self.list_topics.setModel(self.proxy_model)
        self.list_topics.setContextMenuPolicy(Qt.CustomContextMenu)
        self.list_topics.customContextMenuRequested.connect(self.__show_context_menu)
        self.button_refresh.clicked.connect(self.__on_refresh_clicked)
        self.button_filter.clicked.connect(self.__on_filter_clicked)
        self.edit_filter.returnPressed.connect(self.__on_filter_clicked)

    def __on_refresh_clicked(self):
        topics = self.ros2_utils.get_running_topics()
        self.model.set_list(topics)

    def __show_context_menu(self, position):
        index = self.list_topics.indexAt(position)

        if not index.isValid():
            return

        menu = QMenu(self)
        action = menu.addAction("Pass to analyze")
        action_result = menu.exec_(self.list_topics.viewport().mapToGlobal(position))
        if action_result == action:
            self.analyze_request.emit(self.proxy_model.itemData(index)[0])

    def __on_filter_clicked(self):
        filter = self.edit_filter.text()
        self.proxy_model.setFilterFixedString(filter)