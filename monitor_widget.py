from monitor_widget_ui import Ui_MonitorWidget
from PyQt5.QtWidgets import QMenu
from ros2_node import Ros2Node
from PyQt5.QtCore import (
    QAbstractListModel,
    Qt,
    QVariant,
    QAbstractTableModel,
    pyqtSignal,
)


class DataItem:
    def __init__(self, name, type):
        self.name = name
        self.value = None
        self.type = type


class RosData:
    def __init__(self, msg, type):
        self.msg = msg
        self.type = type


class MonitorModel(QAbstractTableModel):
    def __init__(self):
        super().__init__()
        self.headers = ["Name", "Value"]
        self.current_selection = None
        self.values = []

    def rowCount(self, index):
        return len(self.values)

    def columnCount(self, index):
        return 2

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid():
            item = self.values[index.row()]
            if role == Qt.DisplayRole:
                if index.column() == 0:
                    return item.name
                if index.column() == 1:
                    return str(item.value)

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                if section < len(self.headers):
                    return self.headers[section]
                else:
                    return QVariant()
            elif orientation == Qt.Vertical:
                # If you want to set vertical headers, you can do it here
                pass
        return QVariant()

    def set_selection(self, index):
        self.current_selection = index

    def remove_item(self):
        if self.index and self.index.isValid():
            self.beginResetModel()

            name = self.values[self.index.row()].name

            for item in self.values.copy():
                if item.name == name:
                    self.values.remove(item)

            self.current_selection = None
            self.endResetModel()

    def add_item(self, name, type):
        self.current_selection = None
        self.beginResetModel()

        self.values.append(DataItem(name, type))

        self.endResetModel()

    def update_values(self, msg, type):
        self.beginResetModel()

        for item in self.values:
            if item.type == type:
                item.value = self.__get_value(msg, item.name)

        self.endResetModel()

    def remove_topic(self, topic):
        topic_type = None
        for item in self.values.copy():
            if item.name == topic:
                topic_type = item.type
                self.values.remove(item)

        for item in self.values:
            if item.type == topic_type:
                return False

        return True

    def has_mointor_for(self, topic):
        for item in self.values:
            if item.name == topic:
                return True

        return False

    def __get_value(self, obj, request):
        fields = request.split(".")[1:]
        val = obj
        for field in fields:
            val = getattr(val, field, None)

        return val


class MonitorForm(Ui_MonitorWidget):
    ros2_data_update = pyqtSignal(RosData)
    status_change = pyqtSignal(str)

    def __init__(self, parent, node: Ros2Node):
        super().__init__()
        self.setupUi(parent)

        self.node = node
        self.node.set_callback(self.__ros2_callback)
        self.node.set_status_callback(self.__ros2_status_callback)

        self.model_monitor = MonitorModel()
        self.table_topics_values.setModel(self.model_monitor)
        self.table_topics_values.setColumnWidth(0, 400)

        self.button_add_topic.clicked.connect(self.__on_topic_add_clicked)
        self.ros2_data_update.connect(self.__on_ros2_update)

        self.edit_topic_name.returnPressed.connect(self.__on_topic_add_clicked)

        self.table_topics_values.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table_topics_values.customContextMenuRequested.connect(
            self.__show_context_menu
        )

    def on_monitor_requested(self, monitor_value):
        self.edit_topic_name.setText(monitor_value)
        self.__on_topic_add_clicked()

    def __on_topic_add_clicked(self):
        topic = self.edit_topic_name.text()
        if not self.model_monitor.has_mointor_for(topic):
            if self.node.ros2_utils.check_field_appearence(topic):
                    ros2_topic = topic.split(".")[0]
                    _, topic_type = self.node.ros2_utils.get_node_type(ros2_topic)
                    self.model_monitor.add_item(topic, topic_type)
                    self.node.add_subscription(ros2_topic)
                    self.status_change.emit(f"Start monitoring the '{topic}'")
            

    def __ros2_callback(self, msg, type):
        self.ros2_data_update.emit(RosData(msg, type))

    def __on_ros2_update(self, ros2_data: RosData):
        self.model_monitor.update_values(ros2_data.msg, ros2_data.type)

    def __show_context_menu(self, position):
        index = self.table_topics_values.indexAt(position)

        if not index.isValid():
            return

        menu = QMenu(self)
        action = menu.addAction("Remove monitor value")
        action_result = menu.exec_(
            self.table_topics_values.viewport().mapToGlobal(position)
        )

        if action_result == action:
            index = self.model_monitor.index(index.row(), 0)
            topic = self.model_monitor.data(index)
            self.__remove_topic_monitoring(topic)
            self.status_change.emit(f"Remove monitoring for '{topic}'")

    def __remove_topic_monitoring(self, topic):
        if self.model_monitor.remove_topic(topic):
            ros2_topic_name = topic.split(".")[0]
            self.node.remove_subscription(ros2_topic_name)
            

    def __ros2_status_callback(self, status):
        self.status_change.emit(status)