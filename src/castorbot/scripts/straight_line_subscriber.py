#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


def callback(msg):
    print("Received velocity command - ", msg.data) 

def main():
    try:
        rospy.init_node('straight_line_subscriber')
        sub = rospy.Subscriber('/castorbot/rear_motor/command', Float64, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()