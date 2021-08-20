#!/usr/local/bin/python3
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'launcher.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QProcess
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtGui import QColor
from create_roscore import Roslaunch
import os
import sys

import message_filters
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
import numpy as np

os.environ["PYTHONUNBUFFERED"] = "1"

class Stream(QtCore.QObject):
    newText = QtCore.pyqtSignal(str)

    def write(self, text):
        self.newText.emit(str(text))

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
        self.movButton.clicked.connect(self.startMovement)
        self.visionButton = QtWidgets.QPushButton(self.tab)
        self.visionButton.setGeometry(QtCore.QRect(20, 110, 221, 81))
        self.visionButton.setObjectName("visionButton")
        self.visionButton.clicked.connect(self.startVision)
        self.mapButton = QtWidgets.QPushButton(self.tab)
        self.mapButton.setGeometry(QtCore.QRect(20, 210, 221, 81))
        self.mapButton.setObjectName("mapButton")
        self.mapButton.clicked.connect(self.startMapping)
        self.locButton = QtWidgets.QPushButton(self.tab)
        self.locButton.setGeometry(QtCore.QRect(20, 310, 221, 81))
        self.locButton.setObjectName("locButton")
        self.locButton.clicked.connect(self.startLocation)
        self.navButton = QtWidgets.QPushButton(self.tab)
        self.navButton.setGeometry(QtCore.QRect(20, 410, 221, 81))
        self.navButton.setObjectName("navButton")
        self.navButton.clicked.connect(self.startNavigation)
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.tab)
        self.plainTextEdit.setGeometry(QtCore.QRect(270, 10, 511, 481))
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.tabWidget.addTab(self.tab, "")
        self.tab_vision = QtWidgets.QWidget()
        self.tab_vision.setObjectName("tab_vision")
        self.openGLWidget = QtWidgets.QOpenGLWidget(self.tab_vision)
        self.openGLWidget.setGeometry(QtCore.QRect(20, 160, 371, 301))        
        self.openGLWidget.setObjectName("openGLWidget")
        self.frame_maps = QtWidgets.QLabel(self.tab_vision)
        self.frame_maps.setGeometry(QtCore.QRect(410, 160, 371, 301))
        self.frame_maps.setObjectName("frame_maps")
        self.comboBox_Cameras = QtWidgets.QComboBox(self.tab_vision)
        self.comboBox_Cameras.setGeometry(QtCore.QRect(20, 90, 371, 51))
        self.comboBox_Cameras.setObjectName("comboBox_Cameras")
        self.comboBox_maps = QtWidgets.QComboBox(self.tab_vision)
        self.comboBox_maps.setGeometry(QtCore.QRect(410, 90, 371, 51))
        self.comboBox_maps.setObjectName("comboBox_maps")
        self.label = QtWidgets.QLabel(self.tab_vision)
        self.label.setGeometry(QtCore.QRect(20, 50, 341, 17))
        self.label.setObjectName("label")
        self.tabWidget.addTab(self.tab_vision, "")
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
        
        sys.stdout = Stream(newText=self.onUpdateTextOut)
        sys.stderr = Stream(newText=self.onUpdateTextError)
        
        #self.roscore = Roslaunch(["roscore"])
        #self.roscore.run()

    def onUpdateText(self, text, color):    
        old_format = self.plainTextEdit.currentCharFormat()
        color_format = self.plainTextEdit.currentCharFormat()
        color_format.setForeground(color)
        self.plainTextEdit.setCurrentCharFormat(color_format)
        self.plainTextEdit.insertPlainText(text)
        self.plainTextEdit.setCurrentCharFormat(old_format)
        
    def onUpdateTextError(self, text):
        self.onUpdateText(text, QtGui.QColor(255, 0, 0))
    
    def onUpdateTextOut(self, text):
        self.onUpdateText(text, QtGui.QColor(0, 0, 0))
    
    def cam_callback(self, rgb_msg, camera_info):
        rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        # camera_info_K = np.array(camera_info.K).reshape([3, 3])
        # camera_info_D = np.array(camera_info.D)
        # rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)          
        self.image = QtGui.QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
        self.frame_maps.setPixmap(QtGui.QPixmap.fromImage(self.image))
    
    def ros_init(self):
        rospy.init_node('GUI', anonymous=True)
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw/', Image)
        info_sub = message_filters.Subscriber('/camera/color/image_raw/', CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, info_sub], 10, 0.2)
        ts.registerCallback(self.cam_callback)
        #rospy.spin()
        		
    def __del__(self):
        #self.roscore.terminate()
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.movButton.setText(_translate("MainWindow", "Movement"))
        self.visionButton.setText(_translate("MainWindow", "Vision"))
        self.mapButton.setText(_translate("MainWindow", "Mapping"))
        self.locButton.setText(_translate("MainWindow", "Location"))
        self.navButton.setText(_translate("MainWindow", "Navigation"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Initialize"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_vision), _translate("MainWindow", "Visualize"))

    def startMovement(self):
        print('\n\n****************Movement starting****************\n\n')
        self.rosMovement = Roslaunch(["roslaunch create_motion create_motion.launch"])
        self.rosMovement.run()
        
    def startVision(self):
        print('\n\n****************Vision starting****************\n\n')
        self.rosCamera_d435 = Roslaunch(["roslaunch create_camera create_camera_d435.launch"])
        self.rosCamera_t265 = Roslaunch(["roslaunch create_camera create_camera_t265.launch"])
        self.rosCamera_d435.run()
        self.rosCamera_t265.run()
        self.ros_init()
        
    def startMapping(self):
        print('\n\n****************Mapping starting****************\n\n')
        self.rosMovement = Roslaunch(["roslaunch create_rtabmap create_map.launch"])
        self.rosMovement.run()
        
    def startLocation(self):
        print('\n\n****************Location starting****************\n\n')
        self.rosMovement = Roslaunch(["roslaunch create_rtabmap create_loc.launch"])
        self.rosMovement.run()
        
    def startNavigation(self):
        print('\n\n****************Navigation starting****************\n\n')
        self.rosMovement = Roslaunch(["roslaunch create_navigation move_base.launch"])
        self.rosMovement.run()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

