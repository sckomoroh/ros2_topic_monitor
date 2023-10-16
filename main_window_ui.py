# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/main_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName("tabWidget")
        self.monitor_tab = QtWidgets.QWidget()
        self.monitor_tab.setObjectName("monitor_tab")
        self.tabWidget.addTab(self.monitor_tab, "")
        self.structure_tab = QtWidgets.QWidget()
        self.structure_tab.setObjectName("structure_tab")
        self.tabWidget.addTab(self.structure_tab, "")
        self.topics_tab = QtWidgets.QWidget()
        self.topics_tab.setObjectName("topics_tab")
        self.tabWidget.addTab(self.topics_tab, "")
        self.verticalLayout.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(2)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Topics monitor"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.monitor_tab), _translate("MainWindow", "Topic monitor"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.structure_tab), _translate("MainWindow", "Topic structure"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.topics_tab), _translate("MainWindow", "Topic"))
