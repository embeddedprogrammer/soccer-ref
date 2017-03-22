from PyQt4 import QtGui, QtCore

import rospy, rostopic
from soccerref.msg import GameState
from geometry_msgs.msg import Pose2D

from repeated_timer import RepeatedTimer
import os, subprocess, signal

field_width = 3.53  # in meters
field_height = 2.39

# In simulation the ball goes back to home just before this threshold
goal_threshold = field_width/2 + .03

# we know ball is out of the goal if it passes this line
out_of_goal_threshold = 0.3

class RefereeUI(object):
    """docstring for RefereeUI"""

    def __init__(self, ui, sim_mode=True, use_timer=True, competition_mode=False):
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
        self.btn_home_penalty = ui.btnHomePenalty
        self.btn_away_penalty = ui.btnAwayPenalty

        # Score +/- buttons
        self.btn_home_inc_score = ui.btngoal_inc_home
        self.btn_home_dec_score = ui.btngoal_dec_home
        self.btn_away_inc_score = ui.btngoal_inc_away
        self.btn_away_dec_score = ui.btngoal_dec_away

        # Team combo boxes
        self.cmb_teams_home = ui.cmbTeams_home
        self.cmb_teams_away = ui.cmbTeams_away

        # Team Robot settings
        self.spin_bots_home = ui.spinBots_home
        self.spin_bots_away = ui.spinBots_away

        # Team Scoreboard
        self.lbl_score_home = ui.lblhome_score
        self.lbl_score_away = ui.lblaway_score
        self.lbl_team_home = ui.lblhome_team
        self.lbl_team_away = ui.lblaway_team

        # Team groupboxes
        self.gpx_home = ui.frameHome
        self.gpx_away = ui.frameAway

        # Sim mode label
        self.lbl_sim_mode = ui.lblSimMode
        if sim_mode:
            self.lbl_sim_mode.setText('Simulation Mode')
        elif competition_mode:
            self.lbl_sim_mode.setText('Competition Mode')
        else:
            self.lbl_sim_mode.setText('HW Testing Mode')

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

    def populate_team_names(self, sim_mode):
        # a place for team names
        team_list = []

        if sim_mode:
            # this folder is used to launch packages
            catkin_ws_src_folder = os.path.dirname(os.path.abspath(__file__)) + '/../../../'

            def validTeam(obj):
                return os.path.isdir(os.path.join(catkin_ws_src_folder, obj)) and obj not in ['soccersim', 'soccerref']

            # Populate team names (under the fragile assumption that all team names are names of directories)
            team_list = [o for o in os.listdir(catkin_ws_src_folder) if validTeam(o)]

        else:
            # use a yaml file or something
            pass

        self.cmb_teams_home.addItems(team_list)
        self.cmb_teams_away.addItems(team_list)

    def update_scores(self, home_score, away_score):
        self.lbl_score_home.setText(str(home_score))
        self.lbl_score_away.setText(str(away_score))

    def enable_team_settings(self, enable=False):
        self.gpx_home.setEnabled(enable)
        self.gpx_away.setEnabled(enable)

    def reset_team_scoreboards(self, home_team, away_team):
        # Team names on the scoreboards
        self.lbl_team_home.setText(home_team)
        self.lbl_team_away.setText(away_team)

        # Reset scores
        self.update_scores(0, 0)

    def enable_game_play_buttons(self, enable=False):
        self.btn_play.setEnabled(enable)
        self.btn_reset_field.setEnabled(enable)
        self.btn_next_half.setEnabled(enable)
        self.btn_reset_clock.setEnabled(enable)

        self.btn_home_penalty.setEnabled(enable)
        self.btn_away_penalty.setEnabled(enable)

        self.btn_home_inc_score.setEnabled(enable)
        self.btn_home_dec_score.setEnabled(enable)
        self.btn_away_inc_score.setEnabled(enable)
        self.btn_away_dec_score.setEnabled(enable)



class Referee(object):
    """docstring for Referee"""
    def __init__(self, ui, timer_secs, use_timer, sim_mode, competition_mode):
        super(Referee, self).__init__()

        # Setup my UI
        self.ui = RefereeUI(ui, sim_mode, use_timer, competition_mode)

        # Connect to ROS things
        ball_topic = '/ball/truth' if sim_mode else '/vision/ball' # TODO: Create a ref-vision node?
        rospy.Subscriber(ball_topic, Pose2D, self._handle_vision_ball)
        
        self.pub_game_state = rospy.Publisher('game_state', GameState, queue_size=10, latch=True)
        self.sim_mode = sim_mode
        self.game_started = False

        # Create a GameState msg that will be continually updated and published
        self.game_state = GameState()
        self.ballIsStillInGoal = False

        # Set up a 100ms timer event loop
        self.timer = RepeatedTimer(0.1, self._timer_handler)
        self.ui.reset_timer(timer_secs)

        # Populate team names into comboboxes. If sim_mode, these will come from
        # the catkin_ws packages. If not, these need to come from .. a YAML?
        self.ui.populate_team_names(sim_mode)

        # Connect Qt Buttons
        self.ui.btn_play.clicked.connect(self._btn_play)
        self.ui.btn_reset_field.clicked.connect(self._btn_reset_field)
        self.ui.btn_next_half.clicked.connect(self._btn_next_half)
        self.ui.btn_reset_clock.clicked.connect(self._btn_reset_clock)
        self.ui.btn_start_game.clicked.connect(self._btn_start_game)

        # Penalty buttons
        self.ui.btn_home_penalty.clicked.connect(lambda: self._handle_penalty(home=True))
        self.ui.btn_away_penalty.clicked.connect(lambda: self._handle_penalty(home=False))

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

        # Don't leave any loose threads!
        if self.game_started and self.sim_mode:
            # Send a kill signal to all the process groups
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

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
        # If the ball is past the goal_threshold on either side, and if
        # the ball wasn't there before, then it was a goal
        goal = ((msg.x > goal_threshold) or (msg.x < -goal_threshold)) and not self.ballIsStillInGoal

        # Which goal did it go into?
        home_side = (msg.x < 0) # home goal is on the left (negative)
        away_side = (msg.x > 0) # away goal is on the right (positive)

        if goal:
            self.ballIsStillInGoal = True

            # Now lets decide who gets the point, based off home/away and which half we are on
            if home_side ^ self.game_state.second_half:
                self.game_state.away_score += 1

            elif away_side ^ self.game_state.second_half:
                self.game_state.home_score += 1

            # Update UI
            self.ui.update_scores(self.game_state.home_score, self.game_state.away_score)

        # If last we knew, the ball was in the goal but now it's not, update that
        if self.ballIsStillInGoal and abs(msg.x) < out_of_goal_threshold:
            self.ballIsStillInGoal = False

    # =========================================================================
    # Qt Event Callbacks (buttons, etc)
    # =========================================================================

    def _btn_play(self):
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

            # we can also clear any penalties
            self.game_state.home_penalty = False
            self.game_state.away_penalty = False

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
            self.ui.lbl_half.setText('first half')

        else:
            self.game_state.second_half = True
            self.ui.lbl_half.setText('second half')

    def _btn_reset_clock(self):
        # Pause the game if it's being played
        if self.game_state.play:
            self._btn_play()

        self.ui.reset_timer()


    def _btn_start_game(self):
        if not self.game_started:
            self.game_started = True

            # Set game state settings
            self.game_state.home_bot_count = self.ui.spin_bots_home.value()
            self.game_state.away_bot_count = self.ui.spin_bots_away.value()

            # Get selected team names
            home_team = str(self.ui.cmb_teams_home.currentText())
            away_team = str(self.ui.cmb_teams_away.currentText())

            """
            Only launch team roslaunch files if in sim mode.
            """
            if self.sim_mode:
                # Call subprocess using http://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true
                cmd = 'roslaunch soccersim sim.launch home_team:=' + home_team + ' away_team:=' + away_team
                self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

            # Go back to first half if necessary
            if self.game_state.second_half:
                self._btn_next_half()

            # Reset clock
            self._btn_reset_clock()

            # Clear GameState
            self.game_state = GameState()

            # Update UI
            self.ui.enable_team_settings(False)
            self.ui.btn_start_game.setText('Stop Game')
            self.ui.reset_team_scoreboards(home_team, away_team)
            self.ui.enable_game_play_buttons(True)
            

        elif self.game_started:
            self.game_started = False

            if self.sim_mode:
                # Send a kill signal to all the process groups
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

            # stop play
            self.game_state.play = False

            # reset clock
            self.ui.stop_timer()
            
            # Update UI
            self.ui.enable_team_settings(True)
            self.ui.btn_start_game.setText('Start Game')
            self.ui.enable_game_play_buttons(False)
                

    def _handle_score(self, home=True, inc=True):
        # update the global state
        if home:
            self.game_state.home_score += 1 if inc else -1
        else:
            self.game_state.away_score += 1 if inc else -1

        # update the score UI
        self.ui.update_scores(self.game_state.home_score, self.game_state.away_score)


    def _handle_penalty(self, home=True):
        # reset the field
        self._btn_reset_field()

        # let the team know they need to reset in penalty mode
        self.game_state.home_penalty = home
        self.game_state.away_penalty = not home
