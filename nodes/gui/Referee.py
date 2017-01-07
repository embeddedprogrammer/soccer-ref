from PyQt4 import QtGui, QtCore

import rospy, rostopic
from soccer_ref.msg import GameState
from geometry_msgs.msg import Pose2D

from Team import Team
from repeated_timer import RepeatedTimer

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
        # Score +/- buttons
        self.btn_home_inc_score = ui.btngoal_inc_home
        self.btn_home_dec_score = ui.btngoal_dec_home
        self.btn_away_inc_score = ui.btngoal_inc_away
        self.btn_away_dec_score = ui.btngoal_dec_away

        # Sim mode label
        self.lbl_sim_mode = ui.lblSimMode

    def decrement_timer_by_tenth(self):
        pass

    def reset_timer(self, time):
        pass

    def pause_timer(self):
        pass

    def start_timer(self):
        pass

    def is_timer_running(self):
        pass

    def get_timer_seconds(self):
        pass


class Referee(object):
    """docstring for Referee"""
    def __init__(self, ui):
        super(Referee, self).__init__()

        # Setup my UI
        self.ui = RefereeUI(ui)

        # Create these...
        self.home = Team(ui)
        self.away = Team(ui)

        # Set up a 100ms timer event loop
        self.timer = RepeatedTimer(100, self._timer_handler)

        # Connect to ROS things
        rospy.Subscriber('/vision/ball', Pose2D, self._handle_vision_ball)
        self.pub_game_state = rospy.Publisher('/game_state', GameState, queue_size=10, latch=True)

        # Create a GameState msg that will be continually updated and published
        self.game_state_msg = GameState()

        # Connect Qt Buttons
        self.ui.btn_play.clicked.connect(self._btn_play)
        self.ui.btn_reset_field.clicked.connect(self._btn_reset_field)
        self.ui.btn_next_half.clicked.connect(self._btn_next_half)
        self.ui.btn_start_game.clicked.connect(self._btn_start_game)

        # Score +/- buttons
        self.ui.btn_home_inc_score.clicked.connect(lambda: self._handle_score(home=True, inc=True))
        self.ui.btn_home_dec_score.clicked.connect(lambda: self._handle_score(home=True, inc=False))
        self.ui.btn_away_inc_score.clicked.connect(lambda: self._handle_score(home=False, inc=True))
        self.ui.btn_away_dec_score.clicked.connect(lambda: self._handle_score(home=False, inc=False))

    # =========================================================================
    # Public methods
    # =========================================================================

    def close(self):
        self.timer.stop()
        return

    # =========================================================================
    # Timer Event handler
    # =========================================================================

    def _timer_handler(self):
        if self.ui.is_timer_running():
            self.ui.decrement_timer_by_tenth()        

        # send a GameState message
        self.pub_game_state.publish(self.game_state_msg)

    # =========================================================================
    # ROS Event Callbacks (subscribers, event loop)
    # =========================================================================

    def _handle_vision_ball(self, msg):
        if msg.x > goal_threshold:
            self.state.homescore += 1

        elif msg.x < -goal_threshold:
            self.state.awayscore += 1

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