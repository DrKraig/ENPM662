#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class listener:
    def __init__(self):
        self.sub = rospy.Subscriber('/chatter', Float64, self.callback, queue_size=10)
        self.pub = rospy.Publisher('/castorbot/rear_motor/command', Float64, queue_size=10)
        self.pub_right = rospy.Publisher('/castorbot/right_castor_motor/command', Float64, queue_size=10) 
        self.pub_left = rospy.Publisher('/castorbot/left_castor_motor/command', Float64, queue_size=10)
    def callback(self,msg):
        self.pub.publish(msg)
        self.pub_left.publish(0.0)
        self.pub_right.publish(0.0)
        print("Received velocity command - ", msg.data) 

def main():
    try:
        rospy.init_node('straight_line_subscriber')
        bot_commander = listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()