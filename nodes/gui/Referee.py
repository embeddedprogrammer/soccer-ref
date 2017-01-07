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

# we know ball is out of the goal if it passes this line
out_of_goal_threshold = (field_width/2 - 0.05)

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

        # Game timer state
        self.game_timer = {
            'milliseconds': 0,
            'is_running': False
        }

    # ================================= Timer =================================

    def decrement_timer_by_tenth(self):
        self.game_timer['milliseconds'] -= 100

        if self.game_timer['milliseconds'] % 1000 == 0:
            self.update_timer_ui()

    def reset_timer(self, secs):
        self.game_timer['milliseconds'] = secs*1000

        self.update_timer_ui()

    def stop_timer(self):
        self.game_timer['is_running'] = False

    def start_timer(self):
        self.game_timer['is_running'] = True

    def is_timer_done(self):
        return self.game_timer['milliseconds'] == 0

    def is_timer_running(self):
        return self.game_timer['is_running']

    def get_timer_seconds(self):
        ms = self.game_timer['milliseconds']
        return ms/1000

    def update_timer_ui(self):
        ms = self.game_timer['milliseconds']
        secs = (ms / 1000) % 60
        mins = (ms / 1000) / 60
        self.lbl_timer.setText("%d:%02d" % (mins, secs))


class Referee(object):
    """docstring for Referee"""
    def __init__(self, ui):
        super(Referee, self).__init__()

        # Setup my UI
        self.ui = RefereeUI(ui)

        # Create these...
        self.home = Team(ui, team_side='home')
        self.away = Team(ui, team_side='away')

        # Connect to ROS things
        rospy.Subscriber('/vision/ball', Pose2D, self._handle_vision_ball)
        self.pub_game_state = rospy.Publisher('/game_state', GameState, queue_size=10, latch=True)

        # Create a GameState msg that will be continually updated and published
        self.game_state = GameState()
        self.ballIsStillInGoal = False
        settings = rospy.get_param('game_settings', dict()) # returns as a dict

        # Default settings
        self.settings = {}
        if 'match_half_duraction_secs' not in settings:
            self.settings['timer_duration'] = 120
        else:
            self.settings['timer_duration'] = settings['match_half_duration_secs']

        # Set up a 100ms timer event loop
        self.timer = RepeatedTimer(0.1, self._timer_handler)
        self.ui.reset_timer(self.settings['timer_duration'])

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

    # =========================================================================
    # Timer Event handler
    # =========================================================================

    def _timer_handler(self):
        if self.ui.is_timer_done():
            self.ui.stop_timer()

        if self.ui.is_timer_running():
            self.ui.decrement_timer_by_tenth()

        # Add time info to GameState
        self.game_state.remaining_seconds = self.ui.get_timer_seconds()

        # send a GameState message
        self.pub_game_state.publish(self.game_state)

    # =========================================================================
    # ROS Event Callbacks (subscribers, event loop)
    # =========================================================================

    def _handle_vision_ball(self, msg):
        if msg.x > goal_threshold and not self.ballIsStillInGoal:
            self.game_state.homescore += 1

            # update the score UI
            self.home.ui.update_score(self.game_state.homescore)

            # flag so that we only count the goal once
            self.ballIsStillInGoal = True

        elif msg.x < -goal_threshold and not self.ballIsStillInGoal:
            self.game_state.awayscore += 1

            # update the score UI
            self.away.ui.update_score(self.game_state.awayscore)

            # flag so that we only count the goal once
            self.ballIsStillInGoal = True

        elif abs(msg.x) < out_of_goal_threshold:
            self.ballIsStillInGoal = False

    # =========================================================================
    # Qt Event Callbacks (buttons, etc)
    # =========================================================================

    def _btn_play(self):
        # toggle between 'Play' and 'Pause'
        # start/stop timer again
        # GameState.play = true/false
        

        if self.game_state.play:
            # Pause the game and stop the timer
            self.game_state.play = False
            self.ui.stop_timer()

            # Update the UI
            self.ui.btn_play.setText('Play')


        else:
            # Pause the game and stop the timer
            self.game_state.play = True
            self.ui.start_timer()

            # Update the UI
            self.ui.btn_play.setText('Pause')


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
        if home:
            self.game_state.homescore += 1 if inc else -1

            # update the score UI
            self.home.ui.update_score(self.game_state.homescore)
        else:
            self.game_state.awayscore += 1 if inc else -1

            # update the score UI
            self.away.ui.update_score(self.game_state.awayscore)