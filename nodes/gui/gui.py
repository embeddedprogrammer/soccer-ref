#!/usr/bin/env python
import os, sys
from PyQt4 import QtGui, uic, QtCore

import rospy

class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        absdir = os.path.dirname(os.path.abspath(__file__))

        # Load up the UI designed in QtCreator
        uic.loadUi(os.path.join(absdir, 'window.ui'), self)

        # Setup ROS node
        rospy.init_node('referee', anonymous=False)

        # Setup all the GUI and ROS elements for each Ally
        # ally1 = Ally(self, ally=1, active=ally1_active, interval=update_period)
        # ally2 = Ally(self, ally=2, active=ally2_active, interval=update_period)

        # Setup Game State stuff
        # game_state = GameState(self)

if __name__ == '__main__':
    # Set up Qt Application Window
    # QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_X11InitThreads)
    app = QtGui.QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # Run the app
    app.exec_()

    # Manually send ROS the shutdown signal so it cleans up nicely
    rospy.signal_shutdown("User closed the GUI")
    
    sys.exit(0)

# from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
# import ipdb; ipdb.set_trace()
