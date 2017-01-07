import copy

from PyQt4 import QtGui, QtCore

import rospy, rostopic
from referee.msg import GameState

class TeamUI(object):
    """docstring for TeamUI"""

    def __init__(self, ui, team_side):
        super(TeamUI, self).__init__()

        # Team settings
        self.cmb_teams = getattr(ui, 'cmbTeams_{}'.format(team_side))
        self.spin_bots = getattr(ui, 'spinBots_{}'.format(team_side))

        # Team Scoreboard
        self.lbl_score = getattr(ui, 'lbl{}_score'.format(team_side))
        self.lbl_team = getattr(ui, 'lbl{}_team'.format(team_side))

        # ---------------------------------------------------------------------
        # Initialize the GUI

        # self.init_stuff()

class Team(object):
    """docstring for Team"""
    def __init__(self, ui, team_side='home'):
        super(Team, self).__init__()

        # Setup UI
        self.ui = TeamUI(ui, team_side)

        # Connect Qt Things
        # self.ui.cmb_teams.clicked.connect(self._cmb_teams)
        # self.ui.spin_bots.clicked.connect(self._spin_bots)

    # =========================================================================
    # Qt Event Callbacks (buttons, etc)
    # =========================================================================

    def _cmb_teams(self):
        # update this team info
        pass

    def _spin_bots(self):
        # update this team info
        pass