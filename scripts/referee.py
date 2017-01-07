import rospy
from referee.msg import GameState as GameStateMsg

def talker():
    pub = rospy.Publisher('chatter', GameStateMsg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        msg = GameStateMsg()
        msg.homescore = 0
        msg.awayscore = 0
        msg.play = False
        msg.swapsides = False
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
