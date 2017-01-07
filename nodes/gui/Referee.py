import copy

from PyQt4 import QtGui, QtCore

import rospy, rostopic
from referee.msg import GameState
from geometry_msgs.msg import Twist, Pose2D, Vector3

field_width = 3.40  # in meters
field_height = 2.38

# the ball goes back to home after this threshold
goal_threshold = (field_width/2 + 0.05)

class RefereeUI(object):
    """docstring for RefereeUI"""

    def __init__(self, ui):
        super(RefereeUI, self).__init__()

        # Timer
        self.lbl_timer = ui.lbltimer
        self.lbl_half = ui.lblhalf

        # Buttons
        self.btn_play = ui.btnPlay
        self.btn_reset_field = ui.btnResetField
        self.btn_next_half = ui.btnNextHalf
        self.btn_start_game = ui.btnStartGame
        # Deal with home/away +/- btns here? or in Team?

        # Sim mode label
        self.lbl_sim_mode = ui.lblSimMode


class Referee(object):
    """docstring for Referee"""
    def __init__(self, ui):
        super(Referee, self).__init__()

        print("INIT!!!")

        # Setup my UI
        self.ui = RefereeUI(ui)

        # Create these...
        #self.home = Team(ui)
        #self.away = Team(ui)

        # Connect to ROS things
        rospy.Subscriber('/vision/ball', Pose2D, self._handle_vision_ball)
        self.pub_game_state = rospy.Publisher('/game_state', GameState, queue_size=10)
        self.pub_ball_command = rospy.Publisher('/ball/command', Vector3, queue_size=10)
        rospy.init_node('referee')

        # Connect Qt Buttons
        self.ui.btn_play.clicked.connect(self._btn_play)
        self.ui.btn_reset_field.clicked.connect(self._btn_reset_field)
        self.ui.btn_next_half.clicked.connect(self._btn_next_half)
        self.ui.btn_start_game.clicked.connect(self._btn_start_game)

        # Score +/- buttons
        #self.ui.btn_home_inc_score.clicked.connect(lambda: self._handle_score(home=True, inc=True))
        #self.ui.btn_home_dec_score.clicked.connect(lambda: self._handle_score(home=True, inc=False))
        #self.ui.btn_away_inc_score.clicked.connect(lambda: self._handle_score(home=False, inc=True))
        #self.ui.btn_away_dec_score.clicked.connect(lambda: self._handle_score(home=False, inc=False))
        # Should the referee deal with +/- score buttons?

        # Init state
        self.state = GameState()
        self.state.homescore = 0
        self.state.awayscore = 0
        self.state.play = False
        self.state.swapsides = False

        # Publish initial state
        self._publishState()

    def _publishState(self):
        self.pub_game_state.publish(self.state)

    def _resetBall(self):
        msg = Vector3()
        msg.x = 0
        msg.y = 0
        msg.z = 0.2
        self.pub_ball_command.publish(msg)

    # =========================================================================
    # ROS Event Callbacks (subscribers)
    # =========================================================================

    def _handle_vision_ball(self, msg):
        if msg.x > goal_threshold:
            self.state.homescore += 1
            self._resetBall()
            self._publishState()
        elif msg.x < -goal_threshold:
            self.state.awayscore += 1
            self._resetBall()
            self._publishState()

    # =========================================================================
    # Qt Event Callbacks (buttons, etc)
    # =========================================================================

    def _btn_play(self):
        # toggle between 'Play' and 'Pause'
        # start/stop timer again
        # GameState.play = true/false
        pass

    def _btn_reset_field(self):
        # if necessary, press _btn_play to pause game
        # stop timer
        # GameState.reset = true
        pass

    def _btn_next_half(self):
        # press _btn_reset_field
        # clear score
        # stop and reset timer
        pass

    def _btn_start_game(self):
        # toggle between 'Start Game' and 'New Game'
        # if starting a game, disable groupboxes with team settings
        # load team names
        # deal with launch file here?
        #       well, really you need to call a launch file only in the
        #       event that we are in simulation mode.
        #       else, the teams start there own launch files on their
        #       machines.
        pass

    def _handle_score(self, home=True, inc=True):
        # update the global state
        pass