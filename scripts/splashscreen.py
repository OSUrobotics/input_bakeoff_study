#!/usr/bin/env python
import sys
from PySide import QtGui, QtCore


try:
   import rospy
   argv = rospy.myargv()[1]
   rospy.init_node('splashscreen')
except:
   argv = sys.argv[1]

def keyPressEvent(e):
   # quit on escape or space
   if (e.key() == 16777216) or (e.key() == 32):
      QtGui.QApplication.quit()

def mousePressEvent(e):
    QtGui.QApplication.quit()

app = QtGui.QApplication(argv)
wid = QtGui.QLabel(sys.argv[1])
wid.setFont(QtGui.QFont('Arial', 42))
wid.setAlignment(QtCore.Qt.AlignCenter)
wid.keyPressEvent = keyPressEvent
wid.mousePressEvent = mousePressEvent
wid.showFullScreen()
sys.exit(app.exec_())