# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/jacb711/ros/create_ws/src/create_xl/create_gui/config/launcher.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 801, 561))
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.movButton = QtWidgets.QPushButton(self.tab)
        self.movButton.setGeometry(QtCore.QRect(20, 10, 221, 81))
        self.movButton.setObjectName("movButton")
        self.visionButton = QtWidgets.QPushButton(self.tab)
        self.visionButton.setGeometry(QtCore.QRect(20, 110, 221, 81))
        self.visionButton.setObjectName("visionButton")
        self.mapButton = QtWidgets.QPushButton(self.tab)
        self.mapButton.setGeometry(QtCore.QRect(20, 210, 221, 81))
        self.mapButton.setObjectName("mapButton")
        self.locButton = QtWidgets.QPushButton(self.tab)
        self.locButton.setGeometry(QtCore.QRect(20, 310, 221, 81))
        self.locButton.setObjectName("locButton")
        self.navButton = QtWidgets.QPushButton(self.tab)
        self.navButton.setGeometry(QtCore.QRect(20, 410, 221, 81))
        self.navButton.setObjectName("navButton")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.tab)
        self.plainTextEdit.setGeometry(QtCore.QRect(270, 10, 511, 481))
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.openGLWidget = QtWidgets.QOpenGLWidget(self.tab_2)
        self.openGLWidget.setGeometry(QtCore.QRect(20, 160, 371, 301))
        self.openGLWidget.setObjectName("openGLWidget")
        self.frame = QtWidgets.QFrame(self.tab_2)
        self.frame.setGeometry(QtCore.QRect(410, 160, 371, 301))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.comboBox = QtWidgets.QComboBox(self.tab_2)
        self.comboBox.setGeometry(QtCore.QRect(20, 90, 371, 51))
        self.comboBox.setObjectName("comboBox")
        self.comboBox_2 = QtWidgets.QComboBox(self.tab_2)
        self.comboBox_2.setGeometry(QtCore.QRect(410, 90, 371, 51))
        self.comboBox_2.setObjectName("comboBox_2")
        self.label = QtWidgets.QLabel(self.tab_2)
        self.label.setGeometry(QtCore.QRect(20, 50, 341, 17))
        self.label.setObjectName("label")
        self.tabWidget.addTab(self.tab_2, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.movButton.setText(_translate("MainWindow", "Movimiento"))
        self.visionButton.setText(_translate("MainWindow", "Visión"))
        self.mapButton.setText(_translate("MainWindow", "Mapeo"))
        self.locButton.setText(_translate("MainWindow", "Localizacion"))
        self.navButton.setText(_translate("MainWindow", "Navegación"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Inicializar"))
        self.label.setText(_translate("MainWindow", "Seleccionar Vista"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Visualizar"))

