from structure_widget_ui import Ui_StructureWidget
from PyQt5.QtCore import Qt, QAbstractItemModel, QModelIndex, pyqtSignal
from PyQt5.QtWidgets import QMenu, QWidget

from ros2_node import Ros2Utils, TopicStructure


class TopicStructureModel(QAbstractItemModel):
    def __init__(self, parent=None):
        super(TopicStructureModel, self).__init__(parent)
        self.rootItem = TopicStructure(None, None)

    def set_root(self, top):
        self.beginResetModel()

        top.parent = self.rootItem
        self.rootItem.children.clear()
        self.rootItem.children.append(top)

        self.endResetModel()

    def rowCount(self, parent=QModelIndex()):
        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()
        return parentItem.childCount()

    def columnCount(self, parent=QModelIndex()):
        return 2  # name and type

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return None

        item = index.internalPointer()

        if role == Qt.DisplayRole:
            if index.column() == 0:
                return item.name
            elif index.column() == 1:
                return item.type

        if role == Qt.UserRole + 1:
            return item

        return None

    def headerData(self, col, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            if col == 0:
                return "Name"
            elif col == 1:
                return "Type"
        return None

    def index(self, row, column, parent=QModelIndex()):
        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        childItem = parentItem.children[row]
        if childItem:
            return self.createIndex(row, column, childItem)
        return QModelIndex()

    def parent(self, index):
        if not index.isValid():
            return QModelIndex()

        childItem = index.internalPointer()
        parentItem = childItem.parent

        if parentItem == self.rootItem or parentItem is None:
            return QModelIndex()

        return self.createIndex(parentItem.row(), 0, parentItem)


class StructureForm(Ui_StructureWidget, QWidget):
    monitor_request = pyqtSignal(str)
    status_change = pyqtSignal(str)

    def __init__(self, parent, ros2_utils: Ros2Utils):
        super().__init__()
        self.setupUi(parent)
        self.model = TopicStructureModel(parent)
        self.tree_topic_fields.setModel(self.model)
        self.tree_topic_fields.setColumnWidth(0, 400)
        self.button_print.clicked.connect(self.__on_analyze_clicked)
        self.edit_topic_name.returnPressed.connect(self.__on_analyze_clicked)

        self.ros2_utils = ros2_utils

        self.tree_topic_fields.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree_topic_fields.customContextMenuRequested.connect(
            self.__show_context_menu
        )

    def on_analyze_requested(self, topic):
        self.edit_topic_name.setText(topic)
        self.__on_analyze_clicked()
        self.tree_topic_fields.expandAll()

    def __on_analyze_clicked(self):
        topic_name = self.edit_topic_name.text()
        topic_type_name, topic_type = self.ros2_utils.get_node_type(topic_name)
        if not topic_type:
            print(f"[DEBUG] Topic type for '{topic_name}' not found")
            self.status_change.emit(f"Topic type for '{topic_name}' not found")
            return

        if not topic_type_name:
            print(f"[DEBUG] Topic type name for '{topic_name}' not found")
            self.status_change.emit(f"Topic type name for '{topic_name}' not found")
            return

        topic_structure = TopicStructure(topic_name, topic_type)
        self.ros2_utils.analyze_type(topic_type, topic_structure)

        self.model.set_root(topic_structure)
        self.status_change.emit(f"Analyze for '{topic_name}' completed")

    def __show_context_menu(self, position):
        index = self.tree_topic_fields.indexAt(position)

        if not index.isValid():
            return

        menu = QMenu(self)
        action = menu.addAction("Pass to monitor")
        action_result = menu.exec_(
            self.tree_topic_fields.viewport().mapToGlobal(position)
        )
        if action_result == action:
            item: TopicStructure = self.model.data(index, Qt.UserRole + 1)
            parent = item.parent
            path = item.name
            while parent.parent:
                path = parent.name + "." + path
                parent = parent.parent

            self.monitor_request.emit(path)
