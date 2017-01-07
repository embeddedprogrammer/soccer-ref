#!/usr/bin/env python

import rospy
from referee.msg import GameState as GameState
from geometry_msgs.msg import Twist, Pose2D, Vector3

field_width = 3.40  # in meters
field_height = 2.38

# the ball goes back to home after this threshold
goal_threshold = (field_width/2 + 0.05)

class Referee:
    def __init__(self):
        self.gameState_pub = rospy.Publisher('/game_state', GameState, queue_size=10)
        self.ballCommand_pub = rospy.Publisher('/ball/command', Vector3, queue_size=10)
        self.sub = rospy.Subscriber('/vision/ball', Pose2D, self.callback)
        rospy.init_node('referee')

        #init state
        self.state = GameState()
        self.state.homescore = 0
        self.state.awayscore = 0
        self.state.play = False
        self.state.swapsides = False

        #publish initial state
        self.publishState()
    def publishState(self):
        self.gameState_pub.publish(self.state)
    def resetBall(self):
        msg = Vector3()
        msg.x = 0
        msg.y = 0
        msg.z = 0.2
        self.ballCommand_pub.publish(msg)
    def callback(self, ball):
        if ball.x > goal_threshold:
            self.state.homescore += 1
            self.resetBall()
            self.publishState()
        elif ball.x < -goal_threshold:
            self.state.awayscore += 1
            self.resetBall()
            self.publishState()
    def run(self):
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            self.publishState()
            rate.sleep()

if __name__ == '__main__':
    try:
        referee = Referee()
        referee.run()
    except rospy.ROSInterruptException:
        pass