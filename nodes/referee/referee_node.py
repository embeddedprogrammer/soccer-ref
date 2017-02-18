#!/usr/bin/env python
import os, sys
from PyQt4 import QtGui, uic, QtCore

import rospy

from Referee import Referee

class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        absdir = os.path.dirname(os.path.abspath(__file__))

        # Load up the UI designed in QtCreator
        uic.loadUi(os.path.join(absdir, 'window.ui'), self)

        # Setup ROS node
        rospy.init_node('referee', anonymous=False)

        # Get parameters
        timer_secs = rospy.get_param('~half_duration_secs', 120)
        use_timer = rospy.get_param('~use_timer', True)
        sim_mode = rospy.get_param('~simulation_mode', True)
        competition_mode = rospy.get_param('~competition_mode', True)

        # Create a ref
        self.ref = Referee(self, timer_secs, use_timer, sim_mode, competition_mode)


    def close(self):
        self.ref.close()


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
    window.close()
    
    sys.exit(0)

# from PyQt4.QtCore import pyqtRemoveInputHook; pyqtRemoveInputHook()
# import ipdb; ipdb.set_trace()
