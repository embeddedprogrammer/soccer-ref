from PyQt4 import QtGui, QtCore

import rospy, rostopic, roslaunch
from soccerref.msg import GameState
from geometry_msgs.msg import Pose2D

from Team import Team
from repeated_timer import RepeatedTimer
import os, subprocess, signal

field_width = 3.40  # in meters
field_height = 2.38

# the ball goes back to home after this threshold
goal_threshold = (field_width/2 + 0.05)

# we know ball is out of the goal if it passes this line
out_of_goal_threshold = (field_width/2 - 0.05)

# this folder is used to launch packages
catkin_ws_src_folder = os.path.dirname(os.path.abspath(__file__)) + '/../../../'

class RefereeUI(object):
    """docstring for RefereeUI"""

    def __init__(self, ui, sim_mode=True, use_timer=True):
        super(RefereeUI, self).__init__()

        # Timer
        self.lbl_timer = ui.lbltimer
        self.lbl_half = ui.lblhalf

        # Buttons
        self.btn_play = ui.btnPlay
        self.btn_reset_field = ui.btnResetField
        self.btn_next_half = ui.btnNextHalf
        self.btn_reset_clock = ui.btnResetClock
        self.btn_start_game = ui.btnStartGame

        # Score +/- buttons
        self.btn_home_inc_score = ui.btngoal_inc_home
        self.btn_home_dec_score = ui.btngoal_dec_home
        self.btn_away_inc_score = ui.btngoal_inc_away
        self.btn_away_dec_score = ui.btngoal_dec_away
        self.cmb_teams_home = ui.cmbTeams_home
        self.cmb_teams_away = ui.cmbTeams_away

        # Sim mode label
        self.lbl_sim_mode = ui.lblSimMode
        if not sim_mode:
            self.lbl_sim_mode.hide()

        # Game timer state
        self.game_timer = {
            'enabled': use_timer,
            'reset_value': 0,
            'milliseconds': 0,
            'is_running': False
        }

    # ================================= Timer =================================

    def decrement_timer_by_tenth(self):
        self.game_timer['milliseconds'] -= 100

        if self.game_timer['milliseconds'] % 1000 == 0:
            self.update_timer_ui()

    def reset_timer(self, secs=0):
        if secs is not 0:
            self.game_timer['reset_value'] = secs

        elif secs is 0 and self.game_timer['reset_value'] is not 0:
            secs = self.game_timer['reset_value']

        self.game_timer['milliseconds'] = secs*1000

        self.update_timer_ui()

    def stop_timer(self):
        self.game_timer['is_running'] = False

    def start_timer(self):
        if self.game_timer['enabled']:
            self.game_timer['is_running'] = True

    def is_timer_done(self):
        if self.game_timer['enabled']:
            return self.game_timer['milliseconds'] == 0

        return False

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

    # =========================================================================


class Referee(object):
    """docstring for Referee"""
    def __init__(self, ui, timer_secs, use_timer, sim_mode):
        super(Referee, self).__init__()

        # Setup my UI
        self.ui = RefereeUI(ui, sim_mode, use_timer)

        # Create these...
        self.home = Team(ui, team_side='home')
        self.away = Team(ui, team_side='away')

        # Connect to ROS things
        rospy.Subscriber('/vision/ball', Pose2D, self._handle_vision_ball)
        self.pub_game_state = rospy.Publisher('/game_state', GameState, queue_size=10, latch=True)
        self.sim_mode = sim_mode
        self.simRunning = False

        # Create a GameState msg that will be continually updated and published
        self.game_state = GameState()
        self.ballIsStillInGoal = False

        # Set up a 100ms timer event loop
        self.timer = RepeatedTimer(0.1, self._timer_handler)
        self.ui.reset_timer(timer_secs)

        # Populate team names (under the fragile assumption that all team names are names of directories)
        def validTeam(obj):
            return os.path.isdir(os.path.join(catkin_ws_src_folder, obj)) and obj not in ['soccersim', 'soccerref']

        team_list = [o for o in os.listdir(catkin_ws_src_folder) if validTeam(o)]
        self.ui.cmb_teams_home.addItems(team_list)
        self.ui.cmb_teams_away.addItems(team_list)

        # Connect Qt Buttons
        self.ui.btn_play.clicked.connect(self._btn_play)
        self.ui.btn_reset_field.clicked.connect(self._btn_reset_field)
        self.ui.btn_next_half.clicked.connect(self._btn_next_half)
        self.ui.btn_reset_clock.clicked.connect(self._btn_reset_clock)
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
            if self.game_state.play:
                self._btn_play()
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
            # Play the game and start the timer
            self.game_state.play = True
            self.ui.start_timer()

            # We don't need to reset the field anymore
            self.game_state.reset_field = False

            # Update the UI
            self.ui.btn_play.setText('Pause')


    def _btn_reset_field(self):
        # if necessary, press _btn_play to pause game
        # stop timer
        if self.game_state.play:
            self._btn_play()

        self.game_state.reset_field = True


    def _btn_next_half(self):
        # press _btn_reset_field
        # clear score
        # stop and reset timer
        self._btn_reset_field()
        self._btn_reset_clock()

        if self.game_state.second_half:
            self.game_state.second_half = False
            self.game_state.swapsides = False
            self.ui.lbl_half.setText('first half')

        else:
            self.game_state.second_half = True
            self.game_state.swapsides = True
            self.ui.lbl_half.setText('second half')

    def _btn_reset_clock(self):
        # Pause the game if it's being played
        if self.game_state.play:
            self._btn_play()

        self.ui.reset_timer()


    def _btn_start_game(self):
        if self.sim_mode:
            # toggle between 'Start Game' and 'New Game'
            # if starting a game, disable groupboxes with team settings
            # load team names
            if not self.simRunning:
                home_team = str(self.ui.cmb_teams_home.currentText())
                away_team = str(self.ui.cmb_teams_away.currentText())
                cmd = 'roslaunch soccersim sim.launch home_team:=' + home_team + ' away_team:=' + away_team

                # Call subprocess using http://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true
                self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
                self.simRunning = True
                self.ui.btn_start_game.setText('Stop Game')

            elif self.simRunning:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)  # Send the signal to all the process groups
                self.simRunning = False
                self.ui.btn_start_game.setText('Start Game')
                

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