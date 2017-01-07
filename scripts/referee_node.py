#!/usr/bin/env python

import rospy
from referee.msg import GameState as GameStateMsg
from geometry_msgs.msg import Twist, Pose2D

class Referee:
    def __init__(self):
        self.pub = rospy.Publisher('/game_state', GameStateMsg, queue_size=10)
        self.sub = rospy.Subscriber('/vision/ball', Pose2D, self.callback)
        rospy.init_node('referee')
    def callback(self, data):
        print("Test!")
    def publishState(self):
        msg = GameStateMsg()
        msg.homescore = 0
        msg.awayscore = 0
        msg.play = False
        msg.swapsides = False
        self.pub.publish(msg)
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

# TODO: PORT TO PYTHON

# #define FIELD_WIDTH         3.40  // in meters
# #define FIELD_HEIGHT        2.38

# // the ball goes back to home after this threshold
# #define GOAL_THRESHOLD      (FIELD_WIDTH/2 + 0.05)

# else if (model->GetWorldPose().pos.x < -GOAL_THRESHOLD)
# {
#     scoreAway++;
#     SoccerBall::resetBallAndPublishScore();
# }
# else if (model->GetWorldPose().pos.x > GOAL_THRESHOLD)
# {
#     scoreHome++;
#     SoccerBall::resetBallAndPublishScore();
# }

